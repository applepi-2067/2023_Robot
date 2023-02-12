// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeRoller;

public class ActivateIntakeRollers extends CommandBase {
 private static IntakeRoller m_IntakeRoller;
 boolean RollersOn1;
  public ActivateIntakeRollers(Boolean RollersOn) {
    m_IntakeRoller = IntakeRoller.getInstance();
    addRequirements(m_IntakeRoller);
    RollersOn1 = RollersOn;
  }

  @Override
  public void initialize() {}
  
  @Override
  public void execute() {
    if (RollersOn1 == true) {
      m_IntakeRoller.setSpeed(0.3);
    }
    else{
      m_IntakeRoller.setSpeed(0.0);}
  }
 
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
