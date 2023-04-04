// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.arm.BlockUntilArmLessThan;
import frc.robot.commands.arm.SetArmExtension;
import frc.robot.commands.arm.ZeroArmPosition;
import frc.robot.commands.claw.ClawClose;
import frc.robot.commands.claw.ClawOpen;
import frc.robot.commands.claw.ClawSensorGrab;
import frc.robot.commands.claw.SetClawBeltSpeed;
import frc.robot.commands.claw.WaitForGamePieceNotInClaw;
import frc.robot.commands.drivetrain.DriveAtVelocity;
import frc.robot.commands.drivetrain.DriveToPosition;
import frc.robot.commands.fielddriving.DriveToAbsolutePosition;
import frc.robot.commands.fielddriving.RotateToAbsoluteAngle;
import frc.robot.commands.lights.SetLightsColor;
import frc.robot.commands.shoulder.InitShoulderZero;
import frc.robot.commands.shoulder.SetShoulderPosition;
import frc.robot.commands.shoulder.ZeroShoulderPosition;
import frc.robot.commands.waist.RotateWaistToFaceAbsolutePosition;
import frc.robot.commands.waist.SetWaistPosition;
import frc.robot.commands.waist.ZeroWaistPosition;
import frc.robot.subsystems.Lights;
import frc.robot.utils.ScoringPoses;

//Auto routine to be used in the top of bottom scoring position, the routine will score the preloaded piece and leave the community
public class PickupAndScore extends SequentialCommandGroup {
  /** Score preload, pick up cube, then score that one! */

  public PickupAndScore() {
    ScoringPoses scoringPoses = Constants.ScoringInfo.scoringPoses;

    addCommands(
      new DriveToPosition(0.0),
      new ClawClose(),
      // Zero arm extension and shoulder angle
      Commands.parallel(
        new InitShoulderZero().andThen(new ZeroShoulderPosition()),
        new ZeroArmPosition().andThen(new SetArmExtension(0.0))
      ),

      // Raise arm and drop the cone
      new ScoreHighAuto(),
      new ClawOpen(),

      // Retract arm into stow position then drive
      new SetLightsColor(Lights.Color.WHITE),

      // Drive to in front of piece, rotate waist to 180 deg
      Commands.parallel(
        new DriveToAbsolutePosition(scoringPoses.getRobotPickupPiecePose2d()),
        new ZeroWaistPosition().andThen(new SetWaistPosition(scoringPoses.getPickupWaistRotationDegrees())),
        new SetArmExtension(0.0),
        new BlockUntilArmLessThan(0.40).andThen(new SetShoulderPosition(-55.0))
      ),

      new RotateToAbsoluteAngle(scoringPoses.getRobotPickupPieceAbsoluteAngleDegrees()),

      // Pickup with 3 second timeout
      Commands.race(
        new WaitCommand(3),
        Commands.sequence(
          new ClawOpen(),
          new SetShoulderPosition(-55.0),
          Commands.parallel(
            new SetArmExtension(0.25),
            new ClawSensorGrab(),
            new DriveAtVelocity(-1.0)
          ),
          new DriveAtVelocity(0.0)
        )
      ),

      new ClawClose(),

      // Stow and drive
      Commands.deadline(
        new DriveToAbsolutePosition(scoringPoses.getTopCubeScoreRobotPose2d()),
        new RotateWaistToFaceAbsolutePosition(scoringPoses.getTopCubeScorePose2d()),
        new SetArmExtension(0.0),
        new SetShoulderPosition(10.0)
      ),

      // Score
      new SetShoulderPosition(8),
      new SetArmExtension(0.730),
      new SetClawBeltSpeed(() -> {return -0.5;}),
      new ClawOpen(),
      Commands.parallel(
        new WaitForGamePieceNotInClaw(),
        new WaitCommand(0.5)
      ),

      // Go back to stow position
      Commands.parallel(
        new SetArmExtension(0.0).andThen(new SetWaistPosition(0)),
        new BlockUntilArmLessThan(0.40).andThen(new SetShoulderPosition(Constants.Poses.SHOULDER_STOW_ANGLE))
      )
    );
  }
}
