// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawBelt;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.ClawBelt;

public class LightingWithSensor extends CommandBase {

  ClawBelt m_claw;
  Lights m_Lights;
  public LightingWithSensor() {
    m_Lights = Lights.getInstance();
    addRequirements(m_Lights);
    m_claw = ClawBelt.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  if (m_claw.isGamePieceInClaw()) {
    m_Lights.white();
  }
  }
  

  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
