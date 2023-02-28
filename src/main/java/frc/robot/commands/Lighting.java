// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.ClawBelt;

public class Lighting extends CommandBase {
  String m_LightMode;
   private static Lights m_Lights;
   private ClawBelt m_claw;

  public Lighting(String LightMode) {
    m_Lights = Lights.getInstance();
    addRequirements(m_Lights);
    m_LightMode = LightMode;
    m_claw = ClawBelt.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if  (m_claw.isGamePieceInClaw()) {
      m_Lights.white();
    } 
    else if (m_LightMode.equals("purple")) {
      m_Lights.purple();
    }
    else if (m_LightMode.equals("yellow")) {
      m_Lights.yellow();
    }
    else {
      m_Lights.off();
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