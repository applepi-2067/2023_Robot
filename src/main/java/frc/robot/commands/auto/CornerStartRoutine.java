// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.robot.subsystems.Arm;
import frc.robot.commands.arm.SetArmExtension;
import frc.robot.commands.drivetrain.DriveToPosition;
import frc.robot.commands.intake.IntakePiece;
import frc.robot.commands.shoulder.SetShoulderPosition;
import frc.robot.commands.waist.SetWaistPosition;
import frc.robot.commands.claw.ClawClose;
import frc.robot.commands.claw.ClawIntake;
import frc.robot.commands.claw.ClawOpen;
import frc.robot.subsystems.ClawGrasp;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class CornerStartRoutine extends SequentialCommandGroup {
  private Arm arm;
  /** Creates a new TwoBall. */
  public CornerStartRoutine(Drivetrain drivetrain, boolean BlueAlliance, boolean isBottomRoutine) { 
    double invertWaistOne;
    if (isBottomRoutine) {
      invertWaistOne = -1;
    } else {
      invertWaistOne = 1;
    }
    double invertWaistTwo;
    if (BlueAlliance) {
      invertWaistTwo = 1;
    } else {
      invertWaistTwo = -1;
    }
    double invertWaist = invertWaistOne * invertWaistTwo;
    addCommands(
      // 2a
      new SetShoulderPosition(80.0),
      Commands.parallel(
        // 3 and b
        new SetShoulderPosition(130.0),
        new SetWaistPosition(-30*invertWaist)
      ),
      // 4
      new SetArmExtension(1.2192), // 48 inches in meters
      // 5
      new ClawOpen(),
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
         new ClawClose()
        )
      ),
        //(Not a written step) arm retracts to pull piece out of intake
        new SetArmExtension(0),
        // 11b
        new SetShoulderPosition(135.0),
        // 12b
        new SetWaistPosition(30*invertWaist),
        // 12a
        new SetArmExtension(1.2192),
        // 13
        new ClawIntake(false)
    );  
  
  }
}

