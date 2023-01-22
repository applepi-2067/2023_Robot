// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANDeviceIDs;

public class Waist extends SubsystemBase {
  private final CANSparkMax m_motor;
  private final SparkMaxPIDController m_pidController;
  private final RelativeEncoder m_encoder;

  // PID Coefficients.
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  /** Creates a new Waist. */
  public Waist() {
    m_motor = new CANSparkMax(CANDeviceIDs.MOTOR_WAIST_ID, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();

    m_pidController = m_motor.getPIDController();
    m_encoder = m_motor.getEncoder();

    // PID Coefficients.
    kP = 0.1; 
    kI = 1e-4;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

     // Set PID coefficients
     m_pidController.setP(kP);
     m_pidController.setI(kI);
     m_pidController.setD(kD);
     m_pidController.setIZone(kIz);
     m_pidController.setFF(kFF);
     m_pidController.setOutputRange(kMinOutput, kMaxOutput);
  }

  /**
   * Set waist rotation.
   * @param degrees
   */
  public void setPosition(double degrees) {
    m_pidController.setReference(degrees, CANSparkMax.ControlType.kPosition);
  }

  /**
   * Set motor speed.
   * @param speed
   */
  public void setSpeed(double speed) {
    m_motor.set(speed);
  }

  /**
   *  Get waist rotation.
   * @return rotation degrees
   */
  public double getPosition() {
    return m_encoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
