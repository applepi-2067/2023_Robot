// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeV2;
import frc.robot.Constants;

public class IntakeV2Suck extends CommandBase {
  private IntakeV2 intake;
  private Double speed;
  private boolean suckToggle = false;

  public IntakeV2Suck(Double s) {
  
  intake = IntakeV2.getInstance();
  speed = s;  

  addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setSuckSpeed(speed);
    if (suckToggle == false) {
      intake.setSuckSpeed(0.0);
    } else {
      if (suckToggle == true) {
        intake.setSuckSpeed(1.0);
      }
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setSuckSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
