// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.robot.commands.arm.BlockUntilArmLessThan;
import frc.robot.commands.arm.SetArmExtension;
import frc.robot.commands.arm.ZeroArmPosition;
import frc.robot.commands.chargestation.DriveBackwardsUntilAngle;
import frc.robot.commands.chargestation.BalanceOnCharge;
import frc.robot.commands.shoulder.BlockUntilShoulderGreaterThan;
import frc.robot.commands.shoulder.InitShoulderZero;
import frc.robot.commands.shoulder.SetShoulderPosition;
import frc.robot.commands.shoulder.ZeroShoulderPosition;
import frc.robot.commands.waist.SetWaistPosition;
import frc.robot.commands.waist.ZeroWaistPosition;
import frc.robot.commands.claw.ClawClose;
import frc.robot.commands.claw.ClawOpen;
import frc.robot.commands.drivetrain.BlockUntilDistanceTraveled;
import frc.robot.commands.estop.StopDrivetrain;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class CenterStartRoutine extends SequentialCommandGroup {
  public CenterStartRoutine() {
    addCommands(
      new ClawClose(),
      // Zero arm extension and shoulder angle
      Commands.parallel(
        new InitShoulderZero().andThen(new ZeroShoulderPosition()),
        new ZeroArmPosition().andThen(new SetArmExtension(0.0))
      ),
      new ScoreHighAuto(),
      new ClawOpen(),
      Commands.deadline(
        new BlockUntilDistanceTraveled(2.7),
        Commands.sequence(
          Commands.parallel(
            new SetArmExtension(0.0),
            new BlockUntilArmLessThan(0.40).andThen(new SetShoulderPosition(-65.0)), // down in front
            new DriveBackwardsUntilAngle()
          ),
          Commands.parallel(
            new BalanceOnCharge(),
            new ZeroWaistPosition().andThen(new WaitCommand(0.5)).andThen(new SetWaistPosition(0))
          )
        )
      ),
      new StopDrivetrain()
    );
  }
}