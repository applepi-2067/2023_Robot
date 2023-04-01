// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.waist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Waist;

public class ScoringWaistControl extends CommandBase {

  private Waist m_waist;
  private double m_angleToRotate;

  /** Creates a new ScoringWaistControl. */
  public ScoringWaistControl(double angleToRotate) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_waist = Waist.getInstance();
    addRequirements(m_waist);
    m_angleToRotate = angleToRotate;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPosition = m_waist.getPosition();
    m_waist.setPosition(currentPosition + m_angleToRotate);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted) {
      // stop moving
      m_waist.setSpeed(0);
    }
    //otherwise, do nothing... i.e. keep holding last commanded position on exit
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
