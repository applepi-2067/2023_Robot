// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeExtensionMotor;
import frc.robot.utils.Util;

public class ZeroTopLeftIntake extends CommandBase {

  private IntakeExtensionMotor intakeExtensionMotor;
  private final double SPEED = -0.6;
  private final double zeroMotorCurrent = 10.0; //AMPS

  private double averaged_current = 0.0;
  private final double AVG_GAIN = 0.8;

  public ZeroTopLeftIntake() {
   intakeExtensionMotor = IntakeExtensionMotor.getInstance();
   
  }

  @Override
  public void initialize() {
    averaged_current = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeExtensionMotor.setSpeedLeft(SPEED);
    if (intakeExtensionMotor.getLeftMotorCurrent() >= zeroMotorCurrent){
      intakeExtensionMotor.setSpeedLeft(0.0);
      intakeExtensionMotor.zeroLeftEncoder();
    }

    averaged_current = Util.runningAverage(intakeExtensionMotor.getLeftMotorCurrent(), averaged_current,AVG_GAIN );
  }

  @Override
  public void end(boolean interrupted) {
    intakeExtensionMotor.zeroLeftEncoder();
  }

 

  @Override
  public boolean isFinished() {
    return (averaged_current >= zeroMotorCurrent);
  }
}
