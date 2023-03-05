// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.lights;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lights;

public class SetLightsColor extends CommandBase {
  Lights.Color m_color;
  private Lights m_lights;

  public SetLightsColor(Lights.Color color) {
    m_lights = Lights.getInstance();
    addRequirements(m_lights);
    m_color = color;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_lights.setColor(m_color);
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