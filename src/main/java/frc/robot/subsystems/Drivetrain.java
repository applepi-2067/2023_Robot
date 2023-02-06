// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.utils.DrivebaseSimFX;
import frc.robot.utils.PigeonHelper;
import frc.robot.utils.TalonFXHelper;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Drivetrain extends SubsystemBase implements Loggable {
  /** Creates a new DriveTrain. */
  private final TalonFXHelper m_leftMotor = new TalonFXHelper(Constants.CANDeviceIDs.DT_MOTOR_LEFT_1_ID);
  private final TalonFXHelper m_rightMotor = new TalonFXHelper(Constants.CANDeviceIDs.DT_MOTOR_RIGHT_1_ID);
  private final TalonFXHelper m_leftMotorFollower = new TalonFXHelper(Constants.CANDeviceIDs.DT_MOTOR_LEFT_2_ID);
  private final TalonFXHelper m_rightMotorFollower = new TalonFXHelper(Constants.CANDeviceIDs.DT_MOTOR_RIGHT_2_ID);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  private final PigeonHelper m_pidgey;

  public static final double TICKS_PER_REV = 2048.0; // one event per edge on each quadrature channel
  public static final double TICKS_PER_100MS = TICKS_PER_REV / 10.0;
  public static final double GEAR_RATIO = 10.0;
  public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(6.0);
  public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI; // meters
  public static final double PIGEON_UNITS_PER_ROTATION = 8192.0;
  public static final double DEGREES_PER_REV = 360.0;
  public static final double PIGEON_UNITS_PER_DEGREE = PIGEON_UNITS_PER_ROTATION / 360;
  public static final double WHEEL_BASE_METERS = Units.inchesToMeters(24.0); // distance between wheels (width) in meters

  private static Drivetrain drivetrain = null;

  DrivebaseSimFX _driveSim;

  public static Drivetrain getInstance() {
    if (drivetrain == null) {
      drivetrain = new Drivetrain();
    }

    return drivetrain;
  }

  private Drivetrain() {
    // Set values to factory default.
    if (RobotContainer.isPracticeBot()){
      m_pidgey = new PigeonHelper(Constants.CANDeviceIDs.PIGEON_IMU_ID);
    }
    else{
      //This is for the 2022 robot testing
      TalonSRX m_pidgeyController = new TalonSRX(11);
      m_pidgey = new PigeonHelper(m_pidgeyController);
    }
    _driveSim = new DrivebaseSimFX(m_leftMotor, m_rightMotor, (WPI_PigeonIMU) m_pidgey);

    m_robotDrive.setSafetyEnabled(false);
    m_leftMotor.configFactoryDefault();
    m_rightMotor.configFactoryDefault();
    m_leftMotorFollower.configFactoryDefault();
    m_rightMotorFollower.configFactoryDefault();

    // Make back motors follow front motor commands.
    m_leftMotorFollower.follow(m_leftMotor);
    m_rightMotorFollower.follow(m_rightMotor);

    // Set motion magic related parameters
    configMotionMagic(m_leftMotor);
    configMotionMagic(m_rightMotor);

    // configure right talon to use pigeon as remote sensor to aid driving straight
    setAuxPigeon(m_rightMotor);

    // Invert right motors so that positive values make robot move forward.
    // configMotionMagic resets the inversion on the motors, so the .setInversion method
    // should come AFTER the configMotionMagic
    m_leftMotor.setInverted(true);
    m_leftMotorFollower.setInverted(true);

    //Show simulated field position
    SmartDashboard.putData("Field", _driveSim.getField());

    // Set parameters for using pigeon as an aux PID input for driving straight
    // m_rightMotor.configSelectedFeedbackSensor(Remo)
    // _rightConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.Pigeon_Yaw;
		// _rightConfig.remoteFilter0.remoteSensorDeviceID = _pidgey.getDeviceID();
  }

  // Move the robot forward with some rotation.
  public void arcadeDrive(double fwd, double rot) {
    m_robotDrive.arcadeDrive(fwd, rot);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run.
  }

  /**
   * 
   * @param setPoint distance in meters (fwd positive)
   */
  public void setSetPointDistance(double setPoint) {
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
   * 
   * @return current position in meters
   */
  public double getDistanceTraveled() {
    // Negative sign because setter is also flipped
    return ticksToMeters(m_rightMotor.getSelectedSensorPosition());
  }

  /**
   * @return current yaw in degrees (CCW is positive)
   */
  @Log
  public double getYaw() {
    return m_pidgey.getYaw();
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

  /**
   * Change all motors to their default mix of brake/coast modes.
   * Should be used for normal match play.
   */
  public void setMotorsBrake() {
    m_leftMotor.setNeutralMode(NeutralMode.Brake);
    m_leftMotorFollower.setNeutralMode(NeutralMode.Brake);
    m_rightMotor.setNeutralMode(NeutralMode.Brake);
    m_rightMotorFollower.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * Change all the drivetrain motor controllers to coast mode.
   * Useful for allowing robot to be manually pushed around the field.
   */
  public void setMotorsCoast() {
    m_leftMotor.setNeutralMode(NeutralMode.Coast);
    m_leftMotorFollower.setNeutralMode(NeutralMode.Coast);
    m_rightMotor.setNeutralMode(NeutralMode.Coast);
    m_rightMotorFollower.setNeutralMode(NeutralMode.Coast);
  }

  private void configMotionMagic(TalonFXHelper _talon) {

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
    _talon.config_kF(Constants.Drivetrain.kSlotIdx, Constants.Drivetrain.kGains.kF, Constants.Drivetrain.kTimeoutMs);
    _talon.config_kP(Constants.Drivetrain.kSlotIdx, Constants.Drivetrain.kGains.kP, Constants.Drivetrain.kTimeoutMs);
    _talon.config_kI(Constants.Drivetrain.kSlotIdx, Constants.Drivetrain.kGains.kI, Constants.Drivetrain.kTimeoutMs);
    _talon.config_kD(Constants.Drivetrain.kSlotIdx, Constants.Drivetrain.kGains.kD, Constants.Drivetrain.kTimeoutMs);

    /* Set acceleration and vcruise velocity - see documentation */
    // Constants stolen from team 2168's 2022 repo
    _talon.configMotionAcceleration((int) (metersPerSecToTicksPer100ms(Units.inchesToMeters(4.0 * 12.0)))); //(distance units per 100 ms) per second
    _talon.configMotionCruiseVelocity((int) (metersPerSecToTicksPer100ms(Units.inchesToMeters(10.0 * 12.0)))); //distance units per 100 ms
    _talon.configMotionSCurveStrength(8);
  

    /* Zero the sensor once on robot boot up */
    _talon.setSelectedSensorPosition(0, Constants.Drivetrain.kPIDLoopIdx, Constants.Drivetrain.kTimeoutMs);
  }


  private void setAuxPigeon(TalonFXHelper motor) {

  }

  @Override
  public void simulationPeriodic() {
    _driveSim.run();
  }
}