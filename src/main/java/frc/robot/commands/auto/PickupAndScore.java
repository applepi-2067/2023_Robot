// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.arm.SetArmExtension;
import frc.robot.commands.arm.ZeroArmPosition;
import frc.robot.commands.claw.ClawClose;
import frc.robot.commands.claw.ClawOpen;
import frc.robot.commands.claw.ClawSensorGrab;
import frc.robot.commands.drivetrain.DriveToPosition;
import frc.robot.commands.drivetrain.RotateToPosition;
import frc.robot.commands.fielddriving.DriveToAbsolutePosition;
import frc.robot.commands.lights.SetLightsColor;
import frc.robot.commands.shoulder.SetShoulderPosition;
import frc.robot.commands.shoulder.ZeroShoulderPosition;
import frc.robot.commands.waist.SetWaistPosition;
import frc.robot.commands.waist.ZeroWaistPosition;
import frc.robot.subsystems.Lights;

//Auto routine to be used in the top of bottom scoring position, the routine will score the preloaded piece and leave the community
public class PickupAndScore extends SequentialCommandGroup {
  /** Score preload, pick up cube, then score that one! */
  public PickupAndScore() {
    // Robot starts facing the grid in the top of bottom position (not in front of charge station)
          // Drive to 6.13, 4.69

    Pose2d topFieldPiecePositionBlue = new Pose2d(6.0, 4.69, new Rotation2d(0.0));
    Pose2d topCubeScorePositionBlue = new Pose2d(1.81, 4.69, new Rotation2d(0.0));


    addCommands(
      new DriveToPosition(0.0),
      new ClawClose(),
      // Zero arm extension and shoulder angle
      Commands.parallel(
        new ZeroShoulderPosition(),
        new ZeroArmPosition().andThen(new SetArmExtension(0.0))
      ),

      // Raise arm and drop the cone
      new ScoreHighAuto(),
      new ClawOpen(),

      // Retract arm into stow position then drive
      new SetArmExtension(0.0),
      Commands.parallel(
        new SetShoulderPosition(Constants.Poses.SHOULDER_STOW_ANGLE),
        new ZeroWaistPosition().andThen(new SetWaistPosition(0.0))
        // new DriveToPosition(-0.5).andThen(new RotateToPosition(180))
      ),
      new SetLightsColor(Lights.Color.PURPLE),

      // Drive to in front of piece
      new DriveToAbsolutePosition(topFieldPiecePositionBlue, 0.1, true),

      // Pickup with 3 second timeout
      Commands.race(
        new WaitCommand(3),  
        new PickupPieceFromGround()
      ),

      // Stow
      // Rotate to face grid and drive back
      Commands.parallel(
        new SetArmExtension(0.0),
        new SetShoulderPosition(-50.0)
        // new RotateToPosition(180)
      ),
      new DriveToAbsolutePosition(topCubeScorePositionBlue, 0.1, false),

      // Score
      new SetWaistPosition(8),
      new SetShoulderPosition(5),
      new SetArmExtension(0.7),
      new ClawOpen(),

      // Go back to stow position
      new SetArmExtension(0.0),
      new SetShoulderPosition(Constants.Poses.SHOULDER_STOW_ANGLE),
      new SetWaistPosition(0)
    );
  }
}
