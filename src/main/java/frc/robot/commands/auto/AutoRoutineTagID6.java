// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.robot.commands.drivetrain.DriveToPosition;
import frc.robot.commands.drivetrain.RotateToPosition;
import frc.robot.commands.shoulder.SetShoulderPosition;
import frc.robot.commands.waist.SetWaistPosition;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoRoutineTagID6 extends SequentialCommandGroup {
  /** Creates a new TwoBall. */
  public AutoRoutineTagID6(Drivetrain drivetrain) { 
    double squareSideLength = 1; //meters 
    double turnAngle = -90;

    addCommands(
      new SetShoulderPosition(80.0),
      new SetWaistPosition(30.0),
      new SetShoulderPosition(135.0),

      
    );  
  
  }
}
