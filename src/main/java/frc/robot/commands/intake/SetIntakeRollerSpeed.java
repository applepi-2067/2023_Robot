// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeRoller;

public class SetIntakeRollerSpeed extends CommandBase {
  private static IntakeRoller m_IntakeRoller;
  double m_speed;

  public SetIntakeRollerSpeed(double speed) {
    m_IntakeRoller = IntakeRoller.getInstance();
    addRequirements(m_IntakeRoller);
    m_speed = speed;
  }

  @Override
  public void initialize() {}
  
  @Override
  public void execute() {
      m_IntakeRoller.setSpeed(m_speed);
  }
 
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
