// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.lights;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lights;

public class BlinkLights extends CommandBase {
  private Lights m_lights;
  private double m_blinkPeriod;

  public BlinkLights(double blinkPeriod) {
    m_lights = Lights.getInstance();
    addRequirements(m_lights);
    m_blinkPeriod = blinkPeriod;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_lights.blink(m_blinkPeriod);
  }
    

  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}