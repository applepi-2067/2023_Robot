// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.robot.commands.arm.SetArmExtension;
import frc.robot.commands.chargestation.DriveForwardUntilAngle;
import frc.robot.commands.chargestation.balanceOnCharge;
import frc.robot.commands.shoulder.SetShoulderPosition;
import frc.robot.commands.waist.SetWaistPosition;
import frc.robot.commands.claw.ClawClose;
import frc.robot.commands.claw.ClawOpen;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CenterStartRoutine extends SequentialCommandGroup {
  /** Creates a new TwoBall. */
  public CenterStartRoutine() {
    addCommands(
        new SetShoulderPosition(130.0),
        new SetArmExtension(1.2192), // 48 inches in meters
        new ClawOpen(),
        new SetArmExtension(0.0),
        new ClawClose(),
        new SetWaistPosition(0),
        new SetShoulderPosition(25.0),
        new DriveForwardUntilAngle(),
        new balanceOnCharge()
    // Gyro to test if level
    );
  }
}