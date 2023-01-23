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
import frc.robot.utils.Gains;

public class Waist extends SubsystemBase {
  private final CANSparkMax m_motor;
  private final SparkMaxPIDController m_pidController;
  private final RelativeEncoder m_encoder;

  // PID Coefficients.
  private Gains gains = new Gains(0.1, 1e-4, 1, 0, 0, 1);

  /** Creates a new Waist. */
  public Waist() {
    m_motor = new CANSparkMax(CANDeviceIDs.MOTOR_WAIST_ID, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();

    m_pidController = m_motor.getPIDController();
    m_encoder = m_motor.getEncoder();

     // Set PID coefficients
     m_pidController.setP(gains.kP);
     m_pidController.setI(gains.kI);
     m_pidController.setD(gains.kD);
     m_pidController.setIZone(gains.kIzone);
     m_pidController.setFF(gains.kF);
     m_pidController.setOutputRange(-gains.kPeakOutput, gains.kPeakOutput);
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
