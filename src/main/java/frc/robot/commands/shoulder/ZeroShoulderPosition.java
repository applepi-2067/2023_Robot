// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shoulder;

public class ZeroShoulderPosition extends CommandBase {

  private Shoulder m_shoulder;
  private double m_currentShoulderPosition;

  public ZeroShoulderPosition(double currentShoulderPosition) {
    m_currentShoulderPosition = currentShoulderPosition;
    // Use addRequirements() here to declare subsystem dependencies.
    m_shoulder = Shoulder.getInstance();
    addRequirements(m_shoulder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shoulder.setEncoderAngle(m_currentShoulderPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
