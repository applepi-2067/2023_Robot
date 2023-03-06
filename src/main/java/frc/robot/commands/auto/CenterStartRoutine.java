// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.robot.commands.arm.BlockUntilArmLessThan;
import frc.robot.commands.arm.SetArmExtension;
import frc.robot.commands.arm.ZeroArmPosition;
import frc.robot.commands.chargestation.DriveBackwardsUntilAngle;
import frc.robot.commands.chargestation.balanceOnCharge;
import frc.robot.commands.shoulder.SetShoulderPosition;
import frc.robot.commands.shoulder.ZeroShoulderPosition;
import frc.robot.commands.waist.SetWaistPosition;
import frc.robot.commands.claw.ClawClose;
import frc.robot.commands.claw.ClawOpen;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CenterStartRoutine extends SequentialCommandGroup {
  public CenterStartRoutine() {
    addCommands(
      new ClawClose(),
      // Zero arm extension and shoulder angle
      Commands.parallel(
        new ZeroShoulderPosition(),
        new ZeroArmPosition().andThen(new SetArmExtension(0.0))
      ),
      new SetShoulderPosition(20.0),  // Forward is 20 deg  -- High scoring position
      new SetArmExtension(0.82), // High scoring position
      new ClawOpen(),
      Commands.parallel(
        new SetArmExtension(0.0),
        new BlockUntilArmLessThan(0.40).andThen(new SetShoulderPosition(-65.0)), // down in front
        new DriveBackwardsUntilAngle()
      ),
      new balanceOnCharge()
    );
  }
}