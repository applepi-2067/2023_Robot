// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.Constants.CANDeviceIDs;
import frc.robot.commands.intake.SetIntakeExtension;
import frc.robot.utils.Util;
import frc.robot.utils.Gains;

public class IntakeExtensionMotor extends SubsystemBase implements Loggable {

  public static IntakeExtensionMotor instance = null;

  private final CANSparkMax m_motorLeft;
  private final CANSparkMax m_motorRight;
  private final RelativeEncoder m_encoderLeft , m_encoderRight;
  private final SparkMaxPIDController m_pidControllerLeft , m_pidControllerRight;
  private static final boolean INVERT_MOTOR_RIGHT = false;
  private static final boolean INVERT_MOTOR_LEFT = true;

  public static final double SPOCKET_PITCH_DIAMETER_METERS = Units.inchesToMeters(1.751);
  public static final double GEAR_RATIO = 5.23 * 3.61 * 2.89; 
  public static final double METERS_PER_REV = Math.PI * SPOCKET_PITCH_DIAMETER_METERS;

  public static double max_voltage_open_loop = 5;
  private final int CURRENT_LIMIT = 10; // Amps

  // PID Coefficients.
  private Gains gains = new Gains(0.1, 0, 0, 0, 0, 0.8);

  public static IntakeExtensionMotor getInstance() {
    if (instance == null) {
      instance = new IntakeExtensionMotor();
    }
    return instance;
  }

  private IntakeExtensionMotor() {
    m_motorLeft = new CANSparkMax(CANDeviceIDs.INTAKE_EXTENSION_MOTOR_LEFT_ID, MotorType.kBrushless);
    m_motorLeft.restoreFactoryDefaults();
    m_motorLeft.setSmartCurrentLimit(CURRENT_LIMIT);
    m_encoderLeft = m_motorLeft.getEncoder();
    m_pidControllerLeft = m_motorLeft.getPIDController();
    m_motorLeft.setInverted(INVERT_MOTOR_LEFT);

    m_motorRight = new CANSparkMax(CANDeviceIDs.INTAKE_EXTENSION_MOTOR_RIGHT_ID, MotorType.kBrushless);
    m_motorRight.restoreFactoryDefaults();
    m_motorRight.setSmartCurrentLimit(CURRENT_LIMIT);
    m_encoderRight = m_motorRight.getEncoder();
    m_pidControllerRight = m_motorRight.getPIDController();
    m_motorRight.setInverted(INVERT_MOTOR_RIGHT);

    // Set PID coefficients
    m_pidControllerLeft.setP(gains.kP);
    m_pidControllerLeft.setI(gains.kI);
    m_pidControllerLeft.setD(gains.kD);
    m_pidControllerLeft.setIZone(gains.kIzone);
    m_pidControllerLeft.setFF(gains.kF);
    m_pidControllerLeft.setOutputRange(-gains.kPeakOutput, gains.kPeakOutput); 

    m_pidControllerRight.setP(gains.kP);
    m_pidControllerRight.setI(gains.kI);
    m_pidControllerRight.setD(gains.kD);
    m_pidControllerRight.setIZone(gains.kIzone);
    m_pidControllerRight.setFF(gains.kF);
    m_pidControllerRight.setOutputRange(-gains.kPeakOutput, gains.kPeakOutput); 
    

  }

  private double metersToMotorRotations(double degrees) {
    return degrees / METERS_PER_REV * GEAR_RATIO;
  }

  private double motorRotationsToMeters(double rotations) {
    return (rotations / GEAR_RATIO) * METERS_PER_REV;
  }

  /**
   * Set motor speed.
   * 
   * @param speed (-1.0 to 1.0)
   */
  public void setSpeed(double speed) {
    speed = Util.limit(speed);
    m_pidControllerLeft.setReference(speed * max_voltage_open_loop, CANSparkMax.ControlType.kVoltage);
    m_pidControllerRight.setReference(speed * max_voltage_open_loop, CANSparkMax.ControlType.kVoltage);
  }

  public void setSpeedLeft(double speed) {  
    speed = Util.limit(speed);
    m_pidControllerLeft.setReference(speed * max_voltage_open_loop, CANSparkMax.ControlType.kVoltage);
  }

  public void setSpeedRight(double speed) {
    speed = Util.limit(speed);
    m_pidControllerRight.setReference(speed * max_voltage_open_loop, CANSparkMax.ControlType.kVoltage);
  }

  public double getRightMotorCurrent() {
    return m_motorRight.getOutputCurrent();
  }

  public double getLeftMotorCurrent() {
    return m_motorLeft.getOutputCurrent();
  }

  public void zeroLeftEncoder() {
    m_encoderLeft.setPosition(0.0);
  }

  public void zeroRightEncoder() {
    m_encoderRight.setPosition(0.0);
  }

  @Log
  public double getLeftPositionMeters() {
    return motorRotationsToMeters(m_encoderLeft.getPosition());
  }

  @Log
  public double getRightPositionMeters() {
    return motorRotationsToMeters(m_encoderRight.getPosition());
  }

  /**
   * Set intake position.
   * 
   * @param meters
   */
  public void setPosition(double meters) {
    m_pidControllerLeft.setReference(metersToMotorRotations(meters), CANSparkMax.ControlType.kPosition);
    m_pidControllerRight.setReference(metersToMotorRotations(meters), CANSparkMax.ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
