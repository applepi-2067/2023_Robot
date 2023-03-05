// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.lights.BlinkLights;
import frc.robot.commands.lights.DisableBlinkLights;
import frc.robot.commands.lights.SetLightsColor;
import frc.robot.subsystems.Lights;


public class ClawSensorGrab extends SequentialCommandGroup {
  

  public ClawSensorGrab() {
    addCommands(
      new BlinkLights(0.2),
      new ClawOpen(),
      new SetClawBeltSpeed(() -> {return -1.0;}),
      new WaitForGamePieceInClaw(),
      new SetLightsColor(Lights.Color.WHITE),
      new ClawClose(),
      new WaitCommand(0.25),
      new SetClawBeltSpeed(() -> {return 0.0;}),
      new DisableBlinkLights()
    );
  }
}
