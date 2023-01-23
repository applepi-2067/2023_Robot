// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  private final WPI_TalonFX m_leftMotor = new WPI_TalonFX(Constants.OperatorConstants.MOTOR_LEFT_1_ID);
  private final WPI_TalonFX m_rightMotor = new WPI_TalonFX(Constants.OperatorConstants.MOTOR_RIGHT_1_ID);
  private final WPI_TalonFX m_leftMotorFollower = new WPI_TalonFX(Constants.OperatorConstants.MOTOR_LEFT_2_ID);
  private final WPI_TalonFX m_rightMotorFollower = new WPI_TalonFX(Constants.OperatorConstants.MOTOR_RIGHT_2_ID);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  public static final double TICKS_PER_REV = 2048.0; // one event per edge on each quadrature channel
  public static final double TICKS_PER_100MS = TICKS_PER_REV / 10.0;
  public static final double GEAR_RATIO = 10.0;
  public static final double WHEEL_DIAMETER = 6.0;
  public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI; // inches
  public static final double PIGEON_UNITS_PER_ROTATION = 8192.0;
  public static final double DEGREES_PER_REV = 360.0;
  public static final double PIGEON_UNITS_PER_DEGREE = PIGEON_UNITS_PER_ROTATION / 360;
  public static final double WHEEL_BASE = 24.0; // distance between wheels (width) in inches

  public Drivetrain() {
    // Set values to factory default.
    m_leftMotor.configFactoryDefault();
    m_rightMotor.configFactoryDefault();
    m_leftMotorFollower.configFactoryDefault();
    m_rightMotorFollower.configFactoryDefault();

    // Make back motors follow front motor commands.
    m_leftMotorFollower.follow(m_leftMotor);
    m_rightMotorFollower.follow(m_rightMotor);  

    // Invert right motors so that positive values make robot move forward.
    m_rightMotor.setInverted(true);
    m_rightMotorFollower.setInverted(true);

    // Set motion magic related parameters
    configMotionMagic(m_leftMotor);
    configMotionMagic(m_rightMotor);
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
   * @param setPoint distance in inches (fwd positive)
   */
  public void setSetPointDistance(double setPoint) {
    double setPointTicks = inchesToTicks(setPoint);

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
   * 
   * @return current position in inches
   */
  public double getDistanceTraveled() {
    return ticksToInches(m_rightMotor.getSelectedSensorPosition());
  }

  private double inchesToTicks(double setpoint) {
    return (setpoint * TICKS_PER_REV * GEAR_RATIO) / WHEEL_CIRCUMFERENCE;
  }

  private double ticksToInches(double setpoint) {
    return (setpoint * WHEEL_CIRCUMFERENCE) / (TICKS_PER_REV * GEAR_RATIO);
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
    _talon.config_kF(Constants.Drivetrain.kSlotIdx, Constants.Drivetrain.kGains.kF, Constants.Drivetrain.kTimeoutMs);
    _talon.config_kP(Constants.Drivetrain.kSlotIdx, Constants.Drivetrain.kGains.kP, Constants.Drivetrain.kTimeoutMs);
    _talon.config_kI(Constants.Drivetrain.kSlotIdx, Constants.Drivetrain.kGains.kI, Constants.Drivetrain.kTimeoutMs);
    _talon.config_kD(Constants.Drivetrain.kSlotIdx, Constants.Drivetrain.kGains.kD, Constants.Drivetrain.kTimeoutMs);

    /* Set acceleration and vcruise velocity - see documentation */
    _talon.configMotionCruiseVelocity(4000, Constants.Drivetrain.kTimeoutMs);
    _talon.configMotionAcceleration(400, Constants.Drivetrain.kTimeoutMs);

    /* Zero the sensor once on robot boot up */
    _talon.setSelectedSensorPosition(0, Constants.Drivetrain.kPIDLoopIdx, Constants.Drivetrain.kTimeoutMs);
  }
}