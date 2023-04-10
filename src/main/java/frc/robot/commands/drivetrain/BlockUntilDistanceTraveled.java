// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class BlockUntilDistanceTraveled extends CommandBase {
  private double m_distanceThresholdMeters;
  private double m_initialMotorDistanceMeters;
  
  public BlockUntilDistanceTraveled(double distanceThresholdMeters) {
    m_distanceThresholdMeters = distanceThresholdMeters;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_initialMotorDistanceMeters = Drivetrain.getInstance().getAverageMotorDistanceMeters();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double distanceTraveled = Drivetrain.getInstance().getAverageMotorDistanceMeters() - m_initialMotorDistanceMeters;
    return Math.abs(distanceTraveled) > m_distanceThresholdMeters;
  }
}
