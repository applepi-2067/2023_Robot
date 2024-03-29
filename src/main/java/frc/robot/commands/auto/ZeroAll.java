package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.SetArmExtension;
import frc.robot.commands.arm.ZeroArmPosition;
import frc.robot.commands.shoulder.InitShoulderZero;
import frc.robot.commands.shoulder.ZeroShoulderPosition;
import frc.robot.commands.waist.SetWaistPosition;
import frc.robot.commands.waist.ZeroWaistPosition;

public class ZeroAll extends SequentialCommandGroup {
    public ZeroAll() {
        addCommands(
            Commands.parallel(
                new InitShoulderZero().andThen(new ZeroShoulderPosition()),
                new ZeroArmPosition().andThen(new SetArmExtension(0.0))
            ),
            Commands.parallel(
                new ZeroWaistPosition().andThen(new WaitCommand(0.5)).andThen(new SetWaistPosition(0))
            )
        );
    }
    
}
