// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Waist;

public class ShoulderDropPosition extends CommandBase {

  private Shoulder m_shoulder;
  private double m_angleToDrop;

  /** Creates a new ShoulderDropPosition. */
  public ShoulderDropPosition(double angleToDrop) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shoulder = Shoulder.getInstance();
    addRequirements(m_shoulder);
    m_angleToDrop = angleToDrop;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPosition = m_shoulder.getPosition();
    m_shoulder.setPosition(currentPosition - m_angleToDrop);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted) {
      // stop moving
      m_shoulder.setSpeed(0);
    }
    //otherwise, do nothing... i.e. keep holding last commanded position on exit
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
