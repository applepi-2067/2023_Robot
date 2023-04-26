// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.robot.commands.arm.BlockUntilArmLessThan;
import frc.robot.commands.arm.SetArmExtension;
import frc.robot.commands.arm.ZeroArmPosition;
import frc.robot.commands.chargestation.DriveVelocityUntilAngle;
import frc.robot.commands.chargestation.DriveVelocityUntilDistance;
import frc.robot.commands.chargestation.BalanceOnCharge;
import frc.robot.commands.shoulder.InitShoulderZero;
import frc.robot.commands.shoulder.SetShoulderPosition;
import frc.robot.commands.shoulder.ZeroShoulderPosition;
import frc.robot.commands.waist.SetWaistPosition;
import frc.robot.commands.waist.ZeroWaistPosition;
import frc.robot.commands.claw.ClawClose;
import frc.robot.commands.claw.ClawOpen;
import frc.robot.commands.drivetrain.BlockUntilDistanceTraveled;
import frc.robot.commands.fielddriving.DriveToAbsolutePosition;
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

      Commands.parallel(
        // Retract arm.
        new SetArmExtension(0.0),

        // Wait and then lower shoulder.
        new BlockUntilDistanceTraveled(2.0).andThen(new SetShoulderPosition(-65.0)),
        
        // Drive backwards to get mobility.
        new DriveVelocityUntilDistance(-0.5, 4.0) // TODO: failsafe
      ),

      // Drive forwards until we get on the charge station.
      new DriveVelocityUntilDistance(0.6, 1.45),
    
      Commands.parallel(
        new BalanceOnCharge(), // TODO: failsafe w/ distance
        new ZeroWaistPosition().andThen(new WaitCommand(0.5)).andThen(new SetWaistPosition(0))
      )
    );
  }
}