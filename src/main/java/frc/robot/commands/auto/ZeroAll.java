package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.arm.SetArmExtension;
import frc.robot.commands.arm.ZeroArmPosition;
import frc.robot.commands.shoulder.SetShoulderPosition;
import frc.robot.commands.shoulder.ZeroShoulderPosition;
import frc.robot.commands.waist.SetWaistPosition;
import frc.robot.commands.waist.ZeroWaistPosition;
import frc.robot.commands.waist.ZeroWaistPositionCoarse;

public class ZeroAll extends SequentialCommandGroup {
    public ZeroAll() {
        addCommands(
            new ZeroShoulderPosition(Constants.ZeroingOffsets.SHOULDER_FRONT_MINIMUM_ANGLE),
            new SetShoulderPosition(Constants.ZeroingOffsets.SHOULDER_FRONT_MINIMUM_ANGLE + 4.0), // Arm must clear a sprocket
            Commands.parallel(
                new ZeroArmPosition(),
                new WaitCommand(0.5).andThen(new SetShoulderPosition(0.0)),
                new ZeroWaistPosition().andThen(new WaitCommand(0.5)).andThen(new SetWaistPosition(0))
            )
        );
    }
    
}
