// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.lights;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lights;

public class DisableBlinkLights extends CommandBase {
  private Lights m_lights;

  public DisableBlinkLights() {
    m_lights = Lights.getInstance();
    addRequirements(m_lights);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_lights.disableBlink();
    m_lights.on();
  }
    

  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}