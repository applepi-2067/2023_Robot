// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chargestation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveForwardUntilAngle extends CommandBase {
  private static Drivetrain m_driveTrain;

  /** Creates a new DriveForwardUntilAngle. */
  public DriveForwardUntilAngle() {
    Drivetrain drivetrain = Drivetrain.getInstance();
    addRequirements(drivetrain);
    m_driveTrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.arcadeDrive(0.5, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.arcadeDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Check if we're at the angle, if so return true
    return m_driveTrain.getPitchDegrees() > 8;
  }
}
