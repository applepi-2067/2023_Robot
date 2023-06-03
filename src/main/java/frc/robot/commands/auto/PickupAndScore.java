// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.arm.BlockUntilArmLessThan;
import frc.robot.commands.arm.SetArmExtension;
import frc.robot.commands.arm.ZeroArmPosition;
import frc.robot.commands.claw.ClawClose;
import frc.robot.commands.claw.ClawOpen;
import frc.robot.commands.claw.SetClawBeltSpeed;
import frc.robot.commands.claw.WaitForGamePieceNotInClaw;
import frc.robot.commands.drivetrain.DriveAtVelocity;
import frc.robot.commands.fielddriving.DriveToAbsolutePosition;
import frc.robot.commands.fielddriving.RotateToAbsoluteAngle;
import frc.robot.commands.lights.SetLightsColor;
import frc.robot.commands.shoulder.InitShoulderZero;
import frc.robot.commands.shoulder.SetShoulderPosition;
import frc.robot.commands.shoulder.ZeroShoulderPosition;
import frc.robot.commands.teleop_auto.GroundPickup;
import frc.robot.commands.waist.RotateWaistToFaceAbsolutePosition;
import frc.robot.commands.waist.SetWaistPosition;
import frc.robot.commands.waist.ZeroWaistPosition;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Lights;
import frc.robot.utils.PresetPoses;

//Auto routine to be used in the top of bottom scoring position, the routine will score the preloaded piece and leave the community
public class PickupAndScore extends SequentialCommandGroup {
  /** Score preload, pick up cube, then score that one! */

  public PickupAndScore(boolean isBlue, boolean isTop) {
    Pose2d m_initalPose2d;
    Pose2d m_cubePickupPose2d;
    Pose2d m_cubeScorePose2d;

    double m_cubePickupWaistRotationDegrees;
    double m_cubePickupRobotAbsoluteAngleDegrees;

    if (isBlue) {
      if (isTop) {
        m_initalPose2d = PresetPoses.InitialPoses.Blue.TOP_POSE2D;
        m_cubePickupPose2d = PresetPoses.AutoPoses.Blue.Top.CUBE_PICKUP_POSE2D;
        m_cubeScorePose2d = PresetPoses.AutoPoses.Blue.Top.CUBE_SCORE_POSE2D;

        m_cubePickupWaistRotationDegrees = PresetPoses.AutoPoses.Blue.Top.CUBE_PICKUP_WAIST_ROTATION_DEGREES;
        m_cubePickupRobotAbsoluteAngleDegrees = PresetPoses.AutoPoses.Blue.Top.CUBE_PICKUP_ROBOT_ABSOLUTE_ANGLE_DEGREES;
      }
      else {
        m_initalPose2d = PresetPoses.InitialPoses.Blue.BOTTOM_POSE2D;
        m_cubePickupPose2d = PresetPoses.AutoPoses.Blue.Bottom.CUBE_PICKUP_POSE2D;
        m_cubeScorePose2d = PresetPoses.AutoPoses.Blue.Bottom.CUBE_SCORE_POSE2D;

        m_cubePickupWaistRotationDegrees = PresetPoses.AutoPoses.Blue.Bottom.CUBE_PICKUP_WAIST_ROTATION_DEGREES;
        m_cubePickupRobotAbsoluteAngleDegrees = PresetPoses.AutoPoses.Blue.Bottom.CUBE_PICKUP_ROBOT_ABSOLUTE_ANGLE_DEGREES;
      }
    }
    else {
      if (isTop) {
        m_initalPose2d = PresetPoses.InitialPoses.Red.TOP_POSE2D;
        m_cubePickupPose2d = PresetPoses.AutoPoses.Red.Top.CUBE_PICKUP_POSE2D;
        m_cubeScorePose2d = PresetPoses.AutoPoses.Red.Top.CUBE_SCORE_POSE2D;

        m_cubePickupWaistRotationDegrees = PresetPoses.AutoPoses.Red.Top.CUBE_PICKUP_WAIST_ROTATION_DEGREES;
        m_cubePickupRobotAbsoluteAngleDegrees = PresetPoses.AutoPoses.Red.Top.CUBE_PICKUP_ROBOT_ABSOLUTE_ANGLE_DEGREES;
      }
      else {
        m_initalPose2d = PresetPoses.InitialPoses.Red.BOTTOM_POSE2D;
        m_cubePickupPose2d = PresetPoses.AutoPoses.Red.Bottom.CUBE_PICKUP_POSE2D;
        m_cubeScorePose2d = PresetPoses.AutoPoses.Red.Bottom.CUBE_SCORE_POSE2D;

        m_cubePickupWaistRotationDegrees = PresetPoses.AutoPoses.Red.Bottom.CUBE_PICKUP_WAIST_ROTATION_DEGREES;
        m_cubePickupRobotAbsoluteAngleDegrees = PresetPoses.AutoPoses.Red.Bottom.CUBE_PICKUP_ROBOT_ABSOLUTE_ANGLE_DEGREES;
      }
    }

    Drivetrain.getInstance().setOdometryPose2d(m_initalPose2d);

    addCommands(
      // Grip cone.
      Commands.parallel(
        new ClawClose(),
        new SetClawBeltSpeed(() -> {return 0.2;})
      ),
      
      // Zero arm extension and shoulder angle
      Commands.parallel(
        new InitShoulderZero().andThen(new ZeroShoulderPosition()),
        new ZeroArmPosition().andThen(new SetArmExtension(0.0))
      ),

      // Score high auto.      
      new ScoreHighAuto(),
      new WaitCommand(0.1),

      new SetLightsColor(Lights.Color.WHITE),

      // Stow and drive.
      Commands.parallel(
        // Retract arm and shoulder.
        new SetArmExtension(0.0),

        // Wait until arm is in safety zone, and then stow and drive back for mobility.
        new BlockUntilArmLessThan(Constants.Poses.ArmExtensions.SAFE_ROTATION).andThen(
          Commands.parallel(
            new SetShoulderPosition(Constants.Poses.ShoulderAngles.STOW),
            new DriveToAbsolutePosition(m_cubePickupPose2d),

            new ZeroWaistPosition().andThen(new SetWaistPosition(m_cubePickupWaistRotationDegrees))
          )
        )
      ),

      new RotateToAbsoluteAngle(m_cubePickupRobotAbsoluteAngleDegrees),

      // Pickup with 3 second timeout
      Commands.race(
        new WaitCommand(3.0),
        Commands.parallel(
          new DriveAtVelocity(-1.0),
          new GroundPickup()
        )
      ),

      Commands.parallel(
        new ClawClose(),
        new SetArmExtension(Constants.Poses.ArmExtensions.RETRACTED),

        new BlockUntilArmLessThan(Constants.Poses.ArmExtensions.SAFE_ROTATION).andThen(
          Commands.parallel(
            new SetShoulderPosition(Constants.Poses.ShoulderAngles.STOW),
            new DriveToAbsolutePosition(m_cubeScorePose2d),
            new RotateWaistToFaceAbsolutePosition(m_cubeScorePose2d)
          )
        )
      ),

      // Score
      new SetShoulderPosition(Constants.Poses.ShoulderAngles.HIGH_CUBE),
      new SetArmExtension(Constants.Poses.ArmExtensions.HIGH),
      new SetClawBeltSpeed(() -> {return -0.5;}),
      new ClawOpen(),

      Commands.parallel(
        new WaitForGamePieceNotInClaw(),
        new WaitCommand(0.5)
      ),
      new SetClawBeltSpeed(() -> {return 0.0;}),

      // Go back to stow position
      new BlockUntilArmLessThan(Constants.Poses.ArmExtensions.SAFE_ROTATION).andThen(
        new SetShoulderPosition(Constants.Poses.ShoulderAngles.STOW)
      )
    );
  }
}
