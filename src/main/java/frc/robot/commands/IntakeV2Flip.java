// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeV2;
import frc.robot.Constants;

public class IntakeV2Flip extends CommandBase {
  private IntakeV2 intake;
  private Double speed;
  private boolean flipToggle = false;

  
  public IntakeV2Flip() {
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if  (!flipToggle) {
      intake.setFlipSpeed(1.0);
    } else {
      if (flipToggle) {
        intake.setFlipSpeed(-1.0);
      }
    }
    flipToggle = !flipToggle;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setFlipSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (intake.getFlipCurrent() >= 20);
  }
}
