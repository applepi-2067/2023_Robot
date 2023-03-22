// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
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
import frc.robot.commands.drivetrain.DriveToPosition;
import frc.robot.commands.drivetrain.RotateToPosition;
import frc.robot.commands.fielddriving.DriveToAbsolutePosition;
import frc.robot.commands.lights.SetLightsColor;
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

  public PickupAndScore(boolean isTop) {
    boolean isBlue = DriverStation.getAlliance().equals(DriverStation.Alliance.Blue);
    ScoringPoses m_scoringPoses = new ScoringPoses(isBlue, isTop);

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
      new SetLightsColor(Lights.Color.WHITE),

      // Drive to in front of piece, rotate waist to 180 deg
      Commands.parallel(
        new DriveToAbsolutePosition(m_scoringPoses.getRobotPickupPiecePose2d(), true),
        new ZeroWaistPosition().andThen(new SetWaistPosition(180.0)),
        new SetArmExtension(0.0),
        new BlockUntilArmLessThan(0.40).andThen(new SetShoulderPosition(Constants.Poses.SHOULDER_STOW_ANGLE))
      ),

      // Pickup with 3 second timeout
      Commands.race(
        new WaitCommand(3),  
        new PickupPieceFromGround().andThen(new SetLightsColor(Lights.Color.WHITE))
      ),

      // Stow and drive
      Commands.deadline(
        new DriveToAbsolutePosition(m_scoringPoses.getTopCubeScoreRobotPose2d(), false),
        new RotateWaistToFaceAbsolutePosition(m_scoringPoses.getTopCubeScorePose2d()),
        new SetArmExtension(0.0),
        new SetShoulderPosition(-50.0)
      ),


      // Score
      new SetShoulderPosition(10),
      new SetArmExtension(0.8),
      new SetClawBeltSpeed(() -> {return 0.0;}),
      new ClawOpen(),

      // Go back to stow position
      new SetArmExtension(0.0),
      new SetShoulderPosition(Constants.Poses.SHOULDER_STOW_ANGLE),
      new SetWaistPosition(0)
    );
  }
}
