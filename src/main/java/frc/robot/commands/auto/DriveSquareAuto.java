// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.robot.commands.drivetrain.DriveToPosition;
import frc.robot.commands.drivetrain.RotateToPosition;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveSquareAuto extends SequentialCommandGroup {
  /** Creates a new TwoBall. */
  public DriveSquareAuto(Drivetrain drivetrain) { 
    double squareSideLength = 1; //meters 
    double turnAngle = -90;

    addCommands(
        new DriveToPosition(drivetrain, squareSideLength),
        new RotateToPosition(drivetrain, turnAngle), // turn 1
        new DriveToPosition(drivetrain, squareSideLength),
        new RotateToPosition(drivetrain, turnAngle), // turn 2
        new DriveToPosition(drivetrain, squareSideLength),
        new RotateToPosition(drivetrain, turnAngle), // turn 3
        new DriveToPosition(drivetrain, squareSideLength),
        new RotateToPosition(drivetrain, turnAngle) // turn back to original position
    );  
  
  }
}
