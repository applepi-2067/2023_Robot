package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ZeroArmPosition;

public class ZeroAll extends SequentialCommandGroup {
    public ZeroAll() {
        addCommands(
            new ZeroArmPosition()
        );
    }
    
}
