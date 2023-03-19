package frc.robot.commands.fielddriving;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Transforms;

public class DriveToTargetOffset extends SequentialCommandGroup {

  public DriveToTargetOffset(int targetID, Pose2d targetOffsetPose) {    
    Drivetrain m_drivetrain = Drivetrain.getInstance();
    addRequirements(m_drivetrain);

    Pose2d absoluteDestinationPose = Transforms.targetRelativePoseToAbsoluteFieldPose(targetID, targetOffsetPose);
    addCommands(
      // Rotate to face destination.
      new RotateToFaceAbsolutePosition(absoluteDestinationPose),

      // Drive to absolute destination coordinate
      new DriveToAbsolutePosition(absoluteDestinationPose),

      // Rotate to end pose angle
      new RotateToAbsoluteAngle(absoluteDestinationPose.getRotation().getDegrees())
    );
  }
}
