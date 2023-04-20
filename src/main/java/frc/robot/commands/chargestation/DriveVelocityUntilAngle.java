// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chargestation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveVelocityUntilAngle extends CommandBase {
  private static Drivetrain m_drivetrain;

  private double m_velocity;
  private double m_angle;

  public DriveVelocityUntilAngle(double velocity, double angle) {
    m_drivetrain = Drivetrain.getInstance();
    addRequirements(m_drivetrain);
    
    m_velocity = velocity;
    m_angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.arcadeDrive(m_velocity, 0);
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setSetPointVelocity(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Check if we're at the angle, if so return true
    return m_drivetrain.getRollDegrees() >= m_angle;
  }
}
