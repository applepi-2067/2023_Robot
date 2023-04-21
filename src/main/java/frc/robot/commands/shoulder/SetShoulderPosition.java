// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shoulder;

public class SetShoulderPosition extends CommandBase {
  private Shoulder m_shoulder;

  private double m_targetPositionDegrees;

  private int m_targetPositionCount;
  private int m_targetPositionCountThreshold = 10;

  /** Creates a new SetShoulderPosition. */
  public SetShoulderPosition(double targetPositionDegrees) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shoulder = Shoulder.getInstance();
    addRequirements(m_shoulder);

    m_targetPositionDegrees = targetPositionDegrees;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_targetPositionCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shoulder.setPosition(m_targetPositionDegrees);

    double currShoulderPosition = m_shoulder.getPosition();
    double shoulderPositionError = Math.abs(m_targetPositionDegrees - currShoulderPosition);
    if (shoulderPositionError < Constants.SetpointTolerances.SHOULDER_ANGLE_TOLERANCE){
      m_targetPositionCount++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_targetPositionCount > m_targetPositionCountThreshold;
  }
}