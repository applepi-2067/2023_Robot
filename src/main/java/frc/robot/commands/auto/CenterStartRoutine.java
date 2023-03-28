// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.robot.commands.arm.BlockUntilArmLessThan;
import frc.robot.commands.arm.SetArmExtension;
import frc.robot.commands.arm.ZeroArmPosition;
import frc.robot.commands.chargestation.DriveForwardUntilAngle;
import frc.robot.commands.chargestation.BalanceOnCharge;
import frc.robot.commands.shoulder.BlockUntilShoulderGreaterThan;
import frc.robot.commands.shoulder.SetShoulderPosition;
import frc.robot.commands.shoulder.ZeroShoulderPosition;
import frc.robot.commands.waist.SetWaistPosition;
import frc.robot.commands.waist.ZeroWaistPosition;
import frc.robot.commands.claw.ClawClose;
import frc.robot.commands.claw.ClawOpen;
import frc.robot.commands.drivetrain.DriveToPosition;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;

public class CenterStartRoutine extends SequentialCommandGroup {
  public CenterStartRoutine()
{
    addCommands(
      new ClawClose(),
      // Zero arm extension and shoulder angle
      Commands.parallel(
        new ZeroShoulderPosition(),
        new ZeroArmPosition().andThen(new SetArmExtension(0.0))
      ),
      new ScoreHighAuto(),
      new ClawOpen(),
      Commands.parallel(
        new SetArmExtension(0.0),
        new BlockUntilArmLessThan(0.40).andThen(new SetShoulderPosition(-65.0)), // down in front
        new DriveToPosition(-3.9624) // Position robot behind chargestation),
      ),
      new DriveForwardUntilAngle(),
      Commands.parallel(
        new BalanceOnCharge(),
        new ZeroWaistPosition().andThen(new WaitCommand(0.5)).andThen(new SetWaistPosition(0))
      )
    );
  }
}