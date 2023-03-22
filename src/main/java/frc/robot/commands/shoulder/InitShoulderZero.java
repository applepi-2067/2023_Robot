// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shoulder;

public class InitShoulderZero extends CommandBase {

  private Shoulder m_shoulder;

  public InitShoulderZero() {
    m_shoulder = Shoulder.getInstance();
    addRequirements(m_shoulder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shoulder.setSpeed(-1.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shoulder.setSpeed(0.0);
  }

  // Returns true when we have raised the arm high enough to lose the sensor
  @Override
  public boolean isFinished() {
    return m_shoulder.zeroSensorTriggered();
  }
}
