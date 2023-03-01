package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.SetArmExtension;
import frc.robot.commands.shoulder.SetShoulderPosition;
import frc.robot.commands.waist.SetWaistPosition;

public class AquiringGamePiecePosition extends SequentialCommandGroup {
    public AquiringGamePiecePosition () {
        addCommands(
            new SetWaistPosition(0.0),
            new SetShoulderPosition(-74.3),
            new SetArmExtension(0.2)
        );
    }
}
