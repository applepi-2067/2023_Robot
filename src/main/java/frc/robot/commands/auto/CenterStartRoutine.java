// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.robot.commands.arm.SetArmExtension;
import frc.robot.commands.arm.ZeroArmPosition;
import frc.robot.commands.chargestation.DriveForwardUntilAngle;
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
        new ZeroAll(),
        new SetShoulderPosition(160.0),  // Forward is 20 deg  -- High scoring position
        new SetArmExtension(0.82), // High scoring position
        new ClawOpen(),
        new SetArmExtension(0.0),
        new ClawClose(),
        new SetShoulderPosition(-65.0), // down in front
        new DriveForwardUntilAngle(),
        new balanceOnCharge()
    // Gyro to test if level
    );
  }
}