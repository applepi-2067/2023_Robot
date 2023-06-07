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
      new SetArmExtension(Constants.Poses.ArmExtensions.RETRACTED),
      new SetShoulderPosition(Constants.Poses.ShoulderAngles.GROUND_PICKUP),
      Commands.parallel(
        new SetArmExtension(Constants.Poses.ArmExtensions.GROUND_PICKUP),
        new ClawSensorGrab()
      ),
      new SetArmExtension(Constants.Poses.ArmExtensions.RETRACTED),
      new SetShoulderPosition(Constants.Poses.ShoulderAngles.STOW)
    );
  }
}
