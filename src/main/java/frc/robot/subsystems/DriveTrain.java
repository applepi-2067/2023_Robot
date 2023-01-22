// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.OperatorConstants.*;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  private final WPI_TalonFX m_leftMotor = new WPI_TalonFX(MOTOR_LEFT_1_ID);
  private final WPI_TalonFX m_rightMotor = new WPI_TalonFX(MOTOR_RIGHT_1_ID);
  private final WPI_TalonFX m_leftMotorFollower = new WPI_TalonFX(MOTOR_LEFT_2_ID);
  private final WPI_TalonFX m_rightMotorFollower = new WPI_TalonFX(MOTOR_RIGHT_2_ID);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  
  public DriveTrain() {
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
  }
  
  // Move the robot forward with some rotation.
  public void arcadeDrive(double fwd, double rot) {
    m_robotDrive.arcadeDrive(fwd, rot);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run.
  }
}