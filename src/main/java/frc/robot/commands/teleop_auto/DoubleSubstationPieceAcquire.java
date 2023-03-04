// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop_auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.SetArmExtension;
import frc.robot.commands.claw.ClawOpen;
import frc.robot.commands.claw.ClawSensorGrab;
import frc.robot.commands.fielddriving.DriveToTargetOffset;
import frc.robot.commands.shoulder.SetShoulderPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DoubleSubstationPieceAcquire extends SequentialCommandGroup {
  /** Creates a new DoubleSubstationPieceAcquire. */
  public DoubleSubstationPieceAcquire() {
    double DISTANCE_FROM_CHARGESTATION_WALL = 1.2;

    double LEFT_RIGHT_SUBSTATION_OFFSET = -0.33;
    double CAMERA_OFFSET = -0.18;  // 18cm offset, remove after camera offset code is integrated !
    int TARGET_ID = 4;

    Pose2d setupRelativePose = new Pose2d(  // X-dimension is normal to target surface
      DISTANCE_FROM_CHARGESTATION_WALL,  // x = perpendicular to target surface
      LEFT_RIGHT_SUBSTATION_OFFSET + CAMERA_OFFSET, // y = left-right from target surface
      new Rotation2d(Math.PI)
    );

    // pose to reset for debug
    Pose2d exitRelativePose = new Pose2d(  // X-dimension is normal to target surface
      3.0,  // x = perpendicular to target surface
      0.0, // y = left-right from target surface
      new Rotation2d(Math.PI / 2)
    );

    addCommands(
      new SetArmExtension(0), // DEBUG
      new SetShoulderPosition(-45), // DEBUG
      
      // Drive to pickup position
      new DriveToTargetOffset(TARGET_ID, setupRelativePose),

      // Pickup piece
      new SetShoulderPosition(8),
      Commands.deadline(
        new ClawSensorGrab(), // Block until we grab a piedce
        new SetArmExtension(0.6)
      ),

      // Stow
      new SetArmExtension(0),
      new SetShoulderPosition(-60),

      // Turn around and exit
      new DriveToTargetOffset(TARGET_ID, exitRelativePose),

      new ClawOpen()  // DEBUG
    );
  }
}
