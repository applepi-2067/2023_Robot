package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveToPosition;
import frc.robot.commands.drivetrain.RotateToPosition;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Transforms;

public class DriveToTargetOffset extends SequentialCommandGroup {

  public DriveToTargetOffset(int targetID, Pose2d targetOffsetPose) {    
    Drivetrain m_drivetrain = Drivetrain.getInstance();
    addRequirements(m_drivetrain);

    Pose2d absoluteDestinationPose = Transforms.targetRelativePoseToAbsoluteFieldPose(targetID, targetOffsetPose);
    Pose2d latestRobotPose = m_drivetrain.getLatestRobotPose2d();

    double x = absoluteDestinationPose.getX() - latestRobotPose.getX();
    double y = absoluteDestinationPose.getY() - latestRobotPose.getY();

    double hypotenuse = Math.sqrt(x * x + y * y);
    double theta = (Math.PI / 2.0) - Math.atan(y / x);

    addCommands(
      // Turn to face destination.
      new RotateToPosition(Units.radiansToDegrees(theta) - latestRobotPose.getRotation().getDegrees()),

      // Drive hypotenuse to destination.
      new DriveToPosition(hypotenuse),

      // Unturn to vertical, and turn to face destination.
      new RotateToPosition(absoluteDestinationPose.getRotation().getDegrees() - Units.radiansToDegrees(theta))
    );
  }
}
