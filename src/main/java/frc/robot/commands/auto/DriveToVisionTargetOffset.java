package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveToPosition;
import frc.robot.commands.drivetrain.RotateToPosition;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

/** A complex auto command that drives forward, releases a hatch, and then drives backward. */
public class DriveToVisionTargetOffset extends SequentialCommandGroup {

  public DriveToVisionTargetOffset(Drivetrain drivetrain, Vision vision, int targetID, Pose2d destinationPoseRelativeToTarget) {    
    Drivetrain m_drivetrain = drivetrain;
    Vision m_vision = vision;

    Pose2d destinationPoseRelativeToCamera = null;
    while (true) {
      destinationPoseRelativeToCamera = m_vision.getCameraToDestPose(targetID, destinationPoseRelativeToTarget);
      System.out.println("Looking for target.     ");
      if (destinationPoseRelativeToCamera != null) {
        break;
      }
    }
    double destinationCameraX = destinationPoseRelativeToCamera.getX();
    double destinationCameraY = destinationPoseRelativeToCamera.getY();
    double angleTurnPoseAngleRadians = destinationPoseRelativeToCamera.getRotation().getRadians();

    double angleToDestinationRadians = Math.atan(destinationCameraY / destinationCameraX);
    double rotationAngleRadians = -angleToDestinationRadians + angleTurnPoseAngleRadians;

    addCommands(
      // Turn to face destination.
      new RotateToPosition(Math.toDegrees(angleToDestinationRadians)),

      // Drive hypotenuse to desination.
      new DriveToPosition(Math.sqrt(destinationCameraX * destinationCameraX + destinationCameraY * destinationCameraY)),

      // Unturn to vertical, and turn to face destination.
      new RotateToPosition(Math.toDegrees(rotationAngleRadians))
    );
  }
}
