// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chargestation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveBackwardsUntilDistance extends CommandBase {
  private static Drivetrain m_drivetrain;

  private double m_startingDistanceMeters;
  private double m_distanceSetpointMeters;

  public DriveBackwardsUntilDistance(double distanceSetpointMeters) {
    m_drivetrain = Drivetrain.getInstance();
    addRequirements(m_drivetrain);

    m_distanceSetpointMeters = distanceSetpointMeters;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startingDistanceMeters = m_drivetrain.getAverageMotorDistanceMeters();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.arcadeDrive(-1.0, 0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setSetPointVelocity(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double distanceTravelled = m_drivetrain.getAverageMotorDistanceMeters() - m_startingDistanceMeters;
    return (Math.abs(distanceTravelled) >= m_distanceSetpointMeters);
  }
}
