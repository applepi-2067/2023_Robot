// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.SetArmExtension;
import frc.robot.commands.arm.ZeroArmPosition;
import frc.robot.commands.claw.ClawClose;
import frc.robot.commands.claw.ClawOpen;
import frc.robot.commands.shoulder.SetShoulderPosition;
import frc.robot.commands.shoulder.ZeroShoulderPosition;

//Auto routine to be used in the top of bottom scoring position, the routine will score the preloaded piece and leave the community
public class ScorePreloadedPiece extends SequentialCommandGroup {
  /** Creates a new ScorePreloadedPiece. */
  public ScorePreloadedPiece() {
    // Robot starts facing the grid in the top of bottom position (not in front of charge station)
    addCommands(
      new ClawClose(),
      // Zero arm extension and shoulder angle
      Commands.parallel(
        new ZeroShoulderPosition(),
        new ZeroArmPosition().andThen(new SetArmExtension(0.0))
      ),

      // Raise arm and drop the cone
      new SetShoulderPosition(13.36),
      new SetArmExtension(0.82),
      new ClawOpen()

      // Retract arm into stow position

      // Drive out of community by driving a set distance
    );
  }
}
