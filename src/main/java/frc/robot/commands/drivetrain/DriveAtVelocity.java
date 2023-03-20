// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveAtVelocity extends CommandBase {
  Drivetrain m_drivetrain;
  double m_velocity_meters_per_second;

  /** Creates a new ArcadeDrive. */
  public DriveAtVelocity(double velocity_meters_per_sec) {
    m_drivetrain = Drivetrain.getInstance();
    m_velocity_meters_per_second = velocity_meters_per_sec;
    addRequirements(m_drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.setSetPointVelocity(m_velocity_meters_per_second, m_velocity_meters_per_second);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
