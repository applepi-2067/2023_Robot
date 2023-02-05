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


public class AutoRoutineTagID8 extends SequentialCommandGroup {
  private Arm arm;
  /** Creates a new TwoBall. */
  public AutoRoutineTagID8(Drivetrain drivetrain) { 
    addCommands(
      // 2a
      new SetShoulderPosition(80.0),
      Commands.parallel(
        // 3a
        new SetWaistPosition(-180.0),
        // 3b
        new SetShoulderPosition(135.0)
      ),
      // 4
      new SetArmExtension(1.2192), // 48 inches in meters
      // 5
      new ClawGraspCommand(true),
      Commands.parallel(
        // 6a
        new SetArmExtension(0),
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
      Commands.parallel( 
        // 9a
        new DriveToPosition(-5.11),
        // 9b
        Commands.sequence( 
        new SetArmExtension(0.1016),
        // 10
        new ClawGraspCommand(true))
        
      ),
        //(Not a written step) arm retracts to pull piece out of intake
        new SetArmExtension(0),
        // 11b
        new SetShoulderPosition(135.0),
        // 12b
        new SetWaistPosition(-30),
        // 12a
        new SetArmExtension(1.2192),
        // 13
        new ClawIntake(false)
      
        //Driving on to the chargepad:
        //new SetArmExtension(0.0),
        //new SetWaistPosition(0),
        //new SetShoulderPosition(25.0),
        //new DriveToPosition(0.1524),
        //new RotateToPosition(90),
        //new DriveToPosition(1.55),
        //new RotateToPosition(-90),
        //new DriveToPosition(2.6),
        //Gyro to test if level
    );  
  
  }
}

