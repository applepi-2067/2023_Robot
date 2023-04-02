package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.claw.ClawOpen;
import frc.robot.commands.claw.ClawSensorGrab;
import frc.robot.commands.shoulder.SetShoulderPosition;

public class PickupOffGround extends SequentialCommandGroup {
  public PickupOffGround() {
    addCommands(
      Commands.parallel(
        new ClawOpen(),
        new SetShoulderPosition(-55)
      ),
      Commands.parallel(
        new SetArmExtension(0.3),
        new ClawSensorGrab()
      ),
      new SetShoulderPosition(Constants.Poses.SHOULDER_STOW_ANGLE)
    );
  }
}
