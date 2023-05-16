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
import frc.robot.commands.teleop_auto.GroundPickup;
import frc.robot.commands.waist.SetWaistPosition;
import frc.robot.commands.waist.ZeroWaistPosition;
import frc.robot.commands.claw.ClawClose;
import frc.robot.commands.claw.ClawOpen;
import frc.robot.commands.drivetrain.BlockUntilDistanceTraveled;
import frc.robot.commands.fielddriving.DriveToAbsolutePosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
        new ZeroWaistPosition().andThen(new SetWaistPosition(0.0)),
        new ZeroArmPosition().andThen(new SetArmExtension(0.0))
      ),
      new ScoreHighAuto(),
      new ClawOpen(),

      //Runs after dropping preload
      Commands.parallel(
        // Retract arm.
        new SetArmExtension(0.0),

        // Stow shoulder and spin waist.
        new BlockUntilArmLessThan(0.2).andThen(
          Commands.parallel(
            new SetShoulderPosition(-65.0),
            new SetWaistPosition(180)
          )
        ),
        
        // Drive backward to pickup position.
        new DriveToAbsolutePosition(new Pose2d(6.0, 2.15, new Rotation2d()), 0.3)
      ),
      
      // Drive backwards and pickup.
      Commands.race(
        new GroundPickup(),
        new DriveVelocityUntilDistance(-0.5, 1.25)
      ),

      // Drive forwards until we get on the charge station.
      new DriveVelocityUntilDistance(0.6, 2.2),

      new BalanceOnCharge() // TODO: failsafe w/ distance
    );
  }
}