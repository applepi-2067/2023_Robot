// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.robot.subsystems.Arm;
import frc.robot.commands.arm.SetArmExtension;
import frc.robot.commands.drivetrain.DriveToPosition;
import frc.robot.commands.drivetrain.RotateToPosition;
import frc.robot.commands.intake.IntakePiece;
import frc.robot.commands.shoulder.SetShoulderPosition;
import frc.robot.commands.waist.SetWaistPosition;
import frc.robot.commands.claw.ClawIntake;
import frc.robot.commands.claw.ClawGraspCommand;
import frc.robot.subsystems.ClawGrasp;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class CenterTagID2And7 extends SequentialCommandGroup {
  private Arm arm;
  /** Creates a new TwoBall. */
  public CenterTagID2And7(Drivetrain drivetrain) { 
    addCommands(
      // 2a
      new SetShoulderPosition(80.0),
      Commands.parallel(
        // 3a
        new SetWaistPosition(150.0),
        // 3b
        new SetShoulderPosition(135.0)
      ),
        new SetArmExtension(1.2192), // 48 inches in meters
        new ClawGraspCommand(true),
        new SetArmExtension(0.0),
        new SetWaistPosition(0),
        new SetShoulderPosition(25.0),
        new DriveToPosition(2.7524)
        //Gyro to test if level
    );
  }
}