// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  private final WPI_TalonFX m_leftMotor = new WPI_TalonFX(1);
  private final WPI_TalonFX m_rightMotor = new WPI_TalonFX(2);
  private final WPI_TalonFX m_leftMotorFollower = new WPI_TalonFX(3);
  private final WPI_TalonFX m_rightMotorFollower = new WPI_TalonFX(4);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  
  public DriveTrain() {
    /* factory default values */
    m_leftMotor.configFactoryDefault(); //what's up?
    m_rightMotor.configFactoryDefault();
    m_leftMotorFollower.configFactoryDefault();
    m_rightMotorFollower.configFactoryDefault();

    //the .follow method tells the secondary motors to follow the commands of their respective motors
    m_leftMotorFollower.follow(m_leftMotor);
    m_rightMotorFollower.follow(m_rightMotor);

    /* flip values so robot moves forward when stick-forward/LEDs-green */
    m_rightMotor.setInverted(true);
    m_rightMotorFollower.setInverted(true);
  }
  
  public void arcadeDrive(double fwd, double rot) {
    m_robotDrive.arcadeDrive(fwd, rot);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}