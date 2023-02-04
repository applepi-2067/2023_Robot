// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.robot.commands.arm.SetArmPosition;
import frc.robot.commands.drivetrain.DriveToPosition;
import frc.robot.commands.drivetrain.RotateToPosition;
import frc.robot.commands.intake.IntakePiece;
import frc.robot.commands.shoulder.SetShoulderPosition;
import frc.robot.commands.waist.SetWaistPosition;
import frc.robot.commands.claw.GrabPiece;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class AutoRoutineTagID6 extends SequentialCommandGroup {
  /** Creates a new TwoBall. */
  public AutoRoutineTagID6(Drivetrain drivetrain) { 
    addCommands(
      new SetShoulderPosition(80.0),
      Commands.parallel(
        new SetWaistPosition(150.0),
        new SetShoulderPosition(135.0)
      ),
      new SetArmPosition(1.2192), // 48 inches in meters
      new GrabPiece(false),
      Commands.parallel(
        new SetArmPosition(0),
        new SetWaistPosition(180.0),
        new SetShoulderPosition(25.0)
      ),
      Commands.deadline(
        new DriveToPosition(5.11), // Distance in meters to travel
        new IntakePiece(true)
      )
      

    );  
  
  }
}
