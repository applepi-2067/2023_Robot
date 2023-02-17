// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Drivetrain extends SubsystemBase implements Loggable{
  /** Creates a new DriveTrain. */
  private static Drivetrain instance = null;
  private final WPI_TalonFX m_leftMotor = new WPI_TalonFX(Constants.CANDeviceIDs.MOTOR_LEFT_1_ID);
  private final WPI_TalonFX m_rightMotor = new WPI_TalonFX(Constants.CANDeviceIDs.MOTOR_RIGHT_1_ID);
  private final WPI_TalonFX m_leftMotorFollower = new WPI_TalonFX(Constants.CANDeviceIDs.MOTOR_LEFT_2_ID);
  private final WPI_TalonFX m_rightMotorFollower = new WPI_TalonFX(Constants.CANDeviceIDs.MOTOR_RIGHT_2_ID);
  private final DifferentialDrive m_drivetrain = new DifferentialDrive(m_leftMotor, m_rightMotor);

  private final SlewRateLimiter m_leftAccelerationLimiter = new SlewRateLimiter(Constants.Drivetrain.MOTOR_ACCELERATION);
  private final SlewRateLimiter m_rightAccelerationLimiter = new SlewRateLimiter(Constants.Drivetrain.MOTOR_ACCELERATION);

  private static PigeonIMU m_pidgey;
  private static TalonSRX m_pidgeyController;

  public static final double TICKS_PER_REV = 2048.0; // one event per edge on each quadrature channel
  public static final double TICKS_PER_100MS = TICKS_PER_REV / 10.0;
  public static final double GEAR_RATIO = 8.0;
  public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);
  public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI; // meters
  public static final double PIGEON_UNITS_PER_ROTATION = 8192.0;
  public static final double DEGREES_PER_REV = 360.0;
  public static final double PIGEON_UNITS_PER_DEGREE = PIGEON_UNITS_PER_ROTATION / 360;
  public static final double WHEEL_BASE_METERS = Units.inchesToMeters(22.0); // distance between wheels (width) in meters

  private final DifferentialDrivePoseEstimator m_odometry;
  private Pose2d m_latestRobotPose2d = new Pose2d();

  private final Field2d m_field = new Field2d();
  
  public static Drivetrain getInstance() {
    if (instance == null) {
      instance = new Drivetrain();
    }

    return instance;
  }

  private Drivetrain() {  // Constructor is private since this class is singleton
    // Set values to factory default.
    if (RobotContainer.isPracticeBot()) {
      m_pidgey = new PigeonIMU(Constants.CANDeviceIDs.PIGEON_IMU_ID);
    }
    else {
      //This is for the 2022 robot testing
      m_pidgeyController = new TalonSRX(11);
      m_pidgey = new PigeonIMU(m_pidgeyController);
    }

    m_drivetrain.setSafetyEnabled(false);

    m_leftMotor.configFactoryDefault();
    m_rightMotor.configFactoryDefault();
    m_leftMotorFollower.configFactoryDefault();
    m_rightMotorFollower.configFactoryDefault();

    // Make back motors follow front motor commands.
    m_leftMotorFollower.follow(m_leftMotor);
    m_rightMotorFollower.follow(m_rightMotor);

    // Set motion magic related parameters.
    configMotionMagic(m_leftMotor);
    configMotionMagic(m_rightMotor);

    // Invert right motors so that positive values make robot move forward.
    // configMotionMagic resets the inversion on the motors, so the .setInversion method
    // should come AFTER the configMotionMagic
    m_leftMotor.setInverted(true);
    m_leftMotorFollower.setInverted(true);

    resetEncoders();
    resetGyro();

    // Initialize pose estimator.
    m_odometry = new DifferentialDrivePoseEstimator(
      new DifferentialDriveKinematics(WHEEL_BASE_METERS), new Rotation2d(getYawRadians()), getRightMotorDistanceMeters(), getLeftMotorDistanceMeters(), new Pose2d(),
      new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.0408, 1.2711, 0.1)
    );

    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void periodic() {
    updateOdometry();
    m_field.setRobotPose(m_latestRobotPose2d);
  }

  /**
   * Tank drive the robot, passing in left and right stick y values.
   * 
   * @param leftStickY: left stick y value on [-1, 1].
   * @param rightStickY: right stick y value on [-1, 1].
   */
  public void tankDrive(double leftStickY, double rightStickY) {
    setSetPointVelocity(
      clampStickY(leftStickY) * Constants.Drivetrain.MAX_DRIVETRAIN_VELOCITY,
      clampStickY(rightStickY) * Constants.Drivetrain.MAX_DRIVETRAIN_VELOCITY
    );
  }

  /**
   * Clamp the stick y-value by zeroing the value on the deadband and rescaling to get full stick dynamic range.
   * 
   * @param stickY: unclamped stick y-value, on [-1, 1].
   * @return: clamped stick y.
   */
  public double clampStickY(double stickY) {
    // Return 0 if the stick is in the deadband.
    if (Math.abs(stickY) < Constants.Drivetrain.DRIVETRAIN_CONTROLLER_DEADBAND) {
      return 0.0;
    }
    // Compensate for the deadband by rescaling stick value to get full stick dynamic range.
    else {
      double shiftedStickY;
      if (stickY > 0.0) {
        shiftedStickY = stickY - Constants.Drivetrain.DRIVETRAIN_CONTROLLER_DEADBAND;
      }
      else {
        shiftedStickY = stickY + Constants.Drivetrain.DRIVETRAIN_CONTROLLER_DEADBAND;
      }
      
      return shiftedStickY / (1.0 - Constants.Drivetrain.DRIVETRAIN_CONTROLLER_DEADBAND);
    }
  }

  /**
   * Set the left and right motor velocities.
   * 
   * @param leftMotorVelocity_MetersPerSec: left motor target velocity in meters per second.
   * @param rightMotorVelocity_MetersPerSec: right motor target velocity in meters per second.
   */
  public void setSetPointVelocity(double leftMotorVelocity_MetersPerSec, double rightMotorVelocity_MetersPerSec) {
    configVelocityControl(m_leftMotor);
    configVelocityControl(m_rightMotor);

    double filteredLeftMotorVelocity_MetersPerSec = m_leftAccelerationLimiter.calculate(leftMotorVelocity_MetersPerSec);
    double filteredRightMotorVelocity_MetersPerSec = m_rightAccelerationLimiter.calculate(rightMotorVelocity_MetersPerSec);

    m_leftMotor.set(TalonFXControlMode.Velocity, metersPerSecToTicksPer100ms(filteredLeftMotorVelocity_MetersPerSec));
    m_rightMotor.set(TalonFXControlMode.Velocity, metersPerSecToTicksPer100ms(filteredRightMotorVelocity_MetersPerSec));
  }

  // Move the robot forward with some rotation.
  public void arcadeDrive(double fwd, double rot) {
    m_drivetrain.arcadeDrive(fwd, rot);
  }

  /**
   * 
   * @param setPoint distance in meters (fwd positive)
   */
  public void setSetPointDistance(double setPoint) {
    configMotionMagic(m_leftMotor);
    configMotionMagic(m_rightMotor);

    double setPointTicks = metersToTicks(setPoint);
    // Flipped the signs to mirror robot driving patterns
    m_leftMotor.set(TalonFXControlMode.MotionMagic, setPointTicks);
    m_rightMotor.set(TalonFXControlMode.MotionMagic, setPointTicks);
  }

  /**
   * Set drivetrain motor positions to zero
   */
  public void resetEncoders() {
    m_leftMotor.getSensorCollection().setIntegratedSensorPosition(0, Constants.Drivetrain.kTimeoutMs);
    m_rightMotor.getSensorCollection().setIntegratedSensorPosition(0, Constants.Drivetrain.kTimeoutMs);

    m_leftMotor.setSelectedSensorPosition(0.0);
    m_rightMotor.setSelectedSensorPosition(0.0);
  }

  /**
   * Reset the gyro yaw values
   */
  public void resetGyro() {
    m_pidgey.setYaw(0, Constants.Drivetrain.kTimeoutMs);
    m_pidgey.setAccumZAngle(0, Constants.Drivetrain.kTimeoutMs);
    return;
  }

  /**
   * @return right motor distance in meters.
   */
  public double getRightMotorDistanceMeters() {
    return ticksToMeters(m_rightMotor.getSelectedSensorPosition());
  }

  /**
   * @return left motor distance in meters.
   */
  public double getLeftMotorDistanceMeters() {
    return ticksToMeters(m_leftMotor.getSelectedSensorPosition());
  }

  public double getAverageMotorDistanceMeters() {
    return (getRightMotorDistanceMeters() + getLeftMotorDistanceMeters()) / 2.0;
  }

  /**
   * @return current yaw in degrees (CCW is positive)
   */
  @Log
  public double getYawDegrees() {
    return m_pidgey.getYaw();
  }

  /**
   * @return current yaw in radians (CCW is positive)
   */
  public double getYawRadians() {
    return Units.degreesToRadians(m_pidgey.getYaw());
  }

  public void updateOdometry() {
    m_latestRobotPose2d = m_odometry.update(
      new Rotation2d(getYawRadians()), getRightMotorDistanceMeters(), getLeftMotorDistanceMeters()
    );
  }

  public void addVisionMeaurement(Pose2d visionEstimatedRobotPose2d, double timestampSeconds) {
    m_odometry.addVisionMeasurement(visionEstimatedRobotPose2d, timestampSeconds);
  }

  public Pose2d getLatestRobotPose2d() {
    return m_latestRobotPose2d;
  }

  private double metersToTicks(double setpoint) {
    return (setpoint * TICKS_PER_REV * GEAR_RATIO) / WHEEL_CIRCUMFERENCE_METERS;
  }

  private double ticksToMeters(double setpoint) {
    return (setpoint * WHEEL_CIRCUMFERENCE_METERS) / (TICKS_PER_REV * GEAR_RATIO);
  }
  
  private double metersPerSecToTicksPer100ms(double setpoint) {
    return metersToTicks(setpoint) / 10.0;
  }

  private void configMotionMagic(WPI_TalonFX _talon) {

    /* Configure Sensor Source for Primary PID */
    _talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.Drivetrain.kPIDLoopIdx,
        Constants.Drivetrain.kTimeoutMs);

    /*
     * set deadband to super small 0.001 (0.1 %).
     * The default deadband is 0.04 (4 %)
     */
    _talon.configNeutralDeadband(0.001, Constants.Drivetrain.kTimeoutMs);

    /**
     * Configure Talon FX Output and Sensor direction accordingly Invert Motor to
     * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
     * sensor to have positive increment when driving Talon Forward (Green LED)
     */
    _talon.setSensorPhase(false);
    _talon.setInverted(false);
    /*
     * Talon FX does not need sensor phase set for its integrated sensor
     * This is because it will always be correct if the selected feedback device is
     * integrated sensor (default value)
     * and the user calls getSelectedSensor* to get the sensor's position/velocity.
     * 
     * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#
     * sensor-phase
     */
    // _talon.setSensorPhase(true);

    /* Set relevant frame periods to be at least as fast as periodic rate */
    _talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.Drivetrain.kTimeoutMs);
    _talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.Drivetrain.kTimeoutMs);

    /* Set the peak and nominal outputs */
    _talon.configNominalOutputForward(0, Constants.Drivetrain.kTimeoutMs);
    _talon.configNominalOutputReverse(0, Constants.Drivetrain.kTimeoutMs);
    _talon.configPeakOutputForward(1, Constants.Drivetrain.kTimeoutMs);
    _talon.configPeakOutputReverse(-1, Constants.Drivetrain.kTimeoutMs);

    /* Set Motion Magic gains in slot0 - see documentation */
    _talon.selectProfileSlot(Constants.Drivetrain.kSlotIdx, Constants.Drivetrain.kPIDLoopIdx);
    _talon.config_kF(Constants.Drivetrain.kSlotIdx, Constants.Drivetrain.kPositionGains.kF, Constants.Drivetrain.kTimeoutMs);
    _talon.config_kP(Constants.Drivetrain.kSlotIdx, Constants.Drivetrain.kPositionGains.kP, Constants.Drivetrain.kTimeoutMs);
    _talon.config_kI(Constants.Drivetrain.kSlotIdx, Constants.Drivetrain.kPositionGains.kI, Constants.Drivetrain.kTimeoutMs);
    _talon.config_kD(Constants.Drivetrain.kSlotIdx, Constants.Drivetrain.kPositionGains.kD, Constants.Drivetrain.kTimeoutMs);

    /* Set acceleration and vcruise velocity - see documentation */
    // Constants stolen from team 2168's 2022 repo
    _talon.configMotionAcceleration((int) (metersPerSecToTicksPer100ms(Units.inchesToMeters(4.0 * 12.0)))); //(distance units per 100 ms) per second
    _talon.configMotionCruiseVelocity((int) (metersPerSecToTicksPer100ms(Units.inchesToMeters(10.0 * 12.0)))); //distance units per 100 ms
    _talon.configMotionSCurveStrength(8);
  

    /* Zero the sensor once on robot boot up */
    _talon.setSelectedSensorPosition(0, Constants.Drivetrain.kPIDLoopIdx, Constants.Drivetrain.kTimeoutMs);
  }

  private void configVelocityControl(WPI_TalonFX _talon) {
    _talon.selectProfileSlot(Constants.Drivetrain.kSlotIdx, Constants.Drivetrain.kPIDLoopIdx);
    _talon.config_kF(Constants.Drivetrain.kSlotIdx, Constants.Drivetrain.kVelocityGains.kF, Constants.Drivetrain.kTimeoutMs);
    _talon.config_kP(Constants.Drivetrain.kSlotIdx, Constants.Drivetrain.kVelocityGains.kP, Constants.Drivetrain.kTimeoutMs);
    _talon.config_kI(Constants.Drivetrain.kSlotIdx, Constants.Drivetrain.kVelocityGains.kI, Constants.Drivetrain.kTimeoutMs);
    _talon.config_kD(Constants.Drivetrain.kSlotIdx, Constants.Drivetrain.kVelocityGains.kD, Constants.Drivetrain.kTimeoutMs);
  }

  public void setMotorsCoast() {
    m_leftMotor.setNeutralMode(NeutralMode.Coast);
    m_leftMotorFollower.setNeutralMode(NeutralMode.Coast);
    m_rightMotor.setNeutralMode(NeutralMode.Coast);
    m_rightMotorFollower.setNeutralMode(NeutralMode.Coast);
  }

  public void setMotorsBrake() {
    m_leftMotor.setNeutralMode(NeutralMode.Brake);
    m_leftMotorFollower.setNeutralMode(NeutralMode.Brake);
    m_rightMotor.setNeutralMode(NeutralMode.Brake);
    m_rightMotorFollower.setNeutralMode(NeutralMode.Brake);
  }
}