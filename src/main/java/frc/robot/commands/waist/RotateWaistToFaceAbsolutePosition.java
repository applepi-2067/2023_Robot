// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.waist;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Waist;

public class RotateWaistToFaceAbsolutePosition extends CommandBase {
  private Waist m_waist;
  private Drivetrain m_drivetrain;
  
  private Pose2d m_absoluteDestinationPose2d;
  private Pose2d m_latestRobotPose2d;

  /** Creates a new RotateWaistToFaceAbsolutePosition. */
  public RotateWaistToFaceAbsolutePosition(Pose2d absoluteDestinationPose2d) {
    m_waist = Waist.getInstance();
    addRequirements(m_waist);

    m_drivetrain = Drivetrain.getInstance();

    m_absoluteDestinationPose2d = absoluteDestinationPose2d;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_latestRobotPose2d = m_drivetrain.getLatestRobotPose2d();

    // Find angle from robot coord to destination coord.
    double xError = m_absoluteDestinationPose2d.getX() - m_latestRobotPose2d.getX();
    double yError = m_absoluteDestinationPose2d.getY() - m_latestRobotPose2d.getY();
    Rotation2d robotToDestinationRotation2d = Rotation2d.fromRadians(Math.atan2(yError, xError));

    // Get drivetrain rotation.
    Rotation2d absoluteDrivetrainHeadingRotation2d = m_latestRobotPose2d.getRotation();

    // Get heading error (destination - waist).
    Rotation2d headingError = robotToDestinationRotation2d.minus(absoluteDrivetrainHeadingRotation2d);

    m_waist.setPosition(headingError.getDegrees());

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
