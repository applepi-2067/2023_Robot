// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.robot.commands.drivetrain.RotateToPosition;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// Test rotations from 5 to 360 degrees at various increments 
public class RotationTest extends SequentialCommandGroup {
  public RotationTest(Drivetrain drivetrain) { 
    addCommands(
        new InstantCommand(() -> System.out.println("Turning 1 degree")),
        new RotateToPosition( 1),
        new WaitCommand(1),
        new InstantCommand(() -> System.out.println("Turning 5 degrees")),
        new RotateToPosition( 5),
        new WaitCommand(1),
        new InstantCommand(() -> System.out.println("Turning 10 degrees")),
        new RotateToPosition( 10),
        new WaitCommand(1),
        new InstantCommand(() -> System.out.println("Turning 30 degrees")),
        new RotateToPosition( 30),
        new WaitCommand(1),
        new InstantCommand(() -> System.out.println("Turning 60 degrees")),
        new RotateToPosition( 60),
        new WaitCommand(1),
        new InstantCommand(() -> System.out.println("Turning 90 degrees")),
        new RotateToPosition( 90),
        new WaitCommand(1),
        new InstantCommand(() -> System.out.println("Turning 180 degrees")),
        new RotateToPosition( 180),
        new WaitCommand(1),
        new InstantCommand(() -> System.out.println("Turning 270 degrees")),
        new RotateToPosition( 270),
        new WaitCommand(1),
        new InstantCommand(() -> System.out.println("Turning 360 degrees")),
        new RotateToPosition( 360),
        new InstantCommand(() -> System.out.println("Rotation test complete"))
        

    );  
  
  }
}
