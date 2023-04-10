// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop_auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arm.SetArmExtension;
import frc.robot.commands.claw.ClawOpen;
import frc.robot.commands.claw.ClawSensorGrab;
import frc.robot.commands.shoulder.SetShoulderPosition;

public class GroundPickup extends SequentialCommandGroup {
  public GroundPickup() {
    addCommands(
      new ClawOpen(),
      new SetArmExtension(0.0),
      new SetShoulderPosition(-55.0),
      Commands.parallel(
        new SetArmExtension(0.25),
        new ClawSensorGrab()
      ),
      new SetArmExtension(0.0),
      new SetShoulderPosition(Constants.Poses.SHOULDER_STOW_ANGLE)
    );
  }
}
