// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.SetArmExtension;
import frc.robot.commands.arm.ZeroArmPosition;
import frc.robot.commands.claw.ClawClose;
import frc.robot.commands.claw.ClawOpen;
import frc.robot.commands.claw.ClawSensorGrab;
import frc.robot.commands.drivetrain.DriveToPosition;
import frc.robot.commands.shoulder.SetShoulderPosition;
import frc.robot.commands.shoulder.ZeroShoulderPosition;
import frc.robot.commands.waist.SetWaistPosition;
import frc.robot.commands.waist.ZeroWaistPosition;

//Auto routine to be used in the top of bottom scoring position, the routine will score the preloaded piece and leave the community
public class ScorePreloadedPiece extends SequentialCommandGroup {
  /** Creates a new ScorePreloadedPiece. */
  public ScorePreloadedPiece() {
    // Robot starts facing the grid in the top of bottom position (not in front of charge station)
    addCommands(
      new DriveToPosition(0.0),
      new ClawClose(),
      // Zero arm extension and shoulder angle
      Commands.parallel(
        new ZeroShoulderPosition(),
        new ZeroArmPosition().andThen(new SetArmExtension(0.0))
      ),

      // Raise arm and drop the cone
      new SetShoulderPosition(20.00),
      new SetArmExtension(0.82),
      new ClawOpen(),

      // Retract arm into stow position then drive
      new SetArmExtension(0.0),
      Commands.parallel(
        new SetShoulderPosition(-60.0),
        new DriveToPosition(-4.254),
        new ZeroWaistPosition().andThen(new SetWaistPosition(0.0))
      )

      // Commands.deadline(
      //   new ClawSensorGrab(),
      //   new SetArmExtension(0.56)
      // ),
      
      // //After piece is grabbed turn to zero 
      // new SetArmExtension(0.0),
      // new SetWaistPosition(0.0),

      // // DEBUG, RESET WAIST FOR TESTING
      // new WaitCommand(1),  // DEBUG ONLY
      // new SetWaistPosition(0)  // DEBUG ONLY
    );
  }
}
