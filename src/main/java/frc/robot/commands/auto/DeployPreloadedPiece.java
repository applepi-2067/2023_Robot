// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arm.SetArmExtension;
import frc.robot.commands.arm.ZeroArmPosition;
import frc.robot.commands.claw.ClawClose;
import frc.robot.commands.claw.ClawOpen;
import frc.robot.commands.shoulder.SetShoulderPosition;
import frc.robot.commands.shoulder.ZeroShoulderPosition;
import frc.robot.commands.waist.SetWaistPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DeployPreloadedPiece extends SequentialCommandGroup {
  /** Creates a new DeployPreloadedPiece. */
  public DeployPreloadedPiece() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ClawClose(),
      Commands.parallel(
        new ZeroShoulderPosition(),
        new ZeroArmPosition().andThen(new SetArmExtension(0.0))
      ),
      new SetShoulderPosition(17.0),
      new SetArmExtension(0.81),
      new SetShoulderPosition(5.0),
      new ClawOpen(),
      new SetArmExtension(0.0),
      new SetShoulderPosition(Constants.ScoringPositions.STOWED_POSITION_SHOULDER_DEGREES)
    );
  }
}
