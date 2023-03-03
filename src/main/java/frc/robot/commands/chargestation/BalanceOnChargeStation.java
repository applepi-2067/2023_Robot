// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chargestation;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveToPosition;


public class BalanceOnChargeStation extends SequentialCommandGroup {

  public BalanceOnChargeStation() {
    
    addCommands( 
    new DriveToPosition(1.0),
      new balanceOnCharge()
    );
  }
}
