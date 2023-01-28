// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.text.DecimalFormat;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Vision;

public class VisionTest extends CommandBase {
  private static Vision m_vision;

  /** Creates a new VisionTest. */
  public VisionTest(Vision vision) {
    addRequirements(vision);
    m_vision = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final int targetID = 3;
    Pose2d destinationTarget = new Pose2d(2, 0, new Rotation2d(Math.toRadians(180)));
    Pose2d destinationCamera = m_vision.getCameraToDestPose(targetID, destinationTarget);

    if (destinationCamera != null) {
      double x = destinationCamera.getX();
      double y = destinationCamera.getY();
      double theta = destinationCamera.getRotation().getDegrees();

      DecimalFormat f = new DecimalFormat("##.00");
      System.out.print("x, y, theta: " + f.format(x) + "\t" + f.format(y) + "\t" + f.format(theta));
    }
    else {
      System.out.println("Target " + targetID + " not tracked");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
