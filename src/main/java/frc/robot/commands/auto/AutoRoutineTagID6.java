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
      // 2a
      new SetShoulderPosition(80.0),
      Commands.parallel(
        // 3a
        new SetWaistPosition(150.0),
        // 3b
        new SetShoulderPosition(135.0)
      ),
      // 4
      new SetArmPosition(1.2192), // 48 inches in meters
      // 5
      new GrabPiece(false),
      Commands.parallel(
        // 6a
        new SetArmPosition(0),
        // 6b
        new SetWaistPosition(0.0),
        // 6c
        new SetShoulderPosition(25.0)
      ),
      Commands.deadline(
        // 7a and 8a
        new DriveToPosition(5.11), // Distance in meters to travel
        // 7b
        new IntakePiece(true)
      ),
        // 9a
        new DriveToPosition(-5.11),
        // 9b
        new SetArmPosition(0.1016),
        
        // skipping step 10 for now (check document in slack)
        new SetShoulderPosition(135.0),
        new SetArmPosition(0.3048),
        new SetWaistPosition(30.0),
        new GrabPiece(false)
    );  
  
  }
}

