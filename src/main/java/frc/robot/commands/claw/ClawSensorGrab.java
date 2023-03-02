// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class ClawSensorGrab extends SequentialCommandGroup {
  

  public ClawSensorGrab() {

    addCommands(
      new ClawOpen(),
      new SetClawBeltSpeed(() -> {return 0.5;}),
      new WaitForGamePieceInClaw(),
      new ClawClose(),
      new WaitCommand(1.0),
      new SetClawBeltSpeed(() -> {return 0.0;})
    );
  }
}
