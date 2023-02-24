// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeExtensionMotor;

public class ZeroTopIntake extends CommandBase {

  private IntakeExtensionMotor intakeExtensionMotor;
  private final double SPEED = 0.2;
  private final double zeroMotorCurrent = 10.0; //AMPS

  public ZeroTopIntake() {
   intakeExtensionMotor = IntakeExtensionMotor.getInstance();
   
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeExtensionMotor.setSpeedLeft(SPEED);
    intakeExtensionMotor.setSpeedRight(SPEED);
    if ()
  }

  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (intakeExtensionMotor.getLeftMotorCurrent() >= zeroMotorCurrent)
     && (intakeExtensionMotor.getRightMotorCurrent() >= zeroMotorCurrent);
  }
}
