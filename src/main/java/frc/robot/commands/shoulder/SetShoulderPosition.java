// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shoulder;

public class SetShoulderPosition extends CommandBase {

  private Shoulder m_shoulder;
  private double targetPositionDegrees = 0.0;

  /** Creates a new SetShoulderPosition. */
  public SetShoulderPosition(double positionDegrees) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shoulder = Shoulder.getInstance();
    addRequirements(m_shoulder);

    targetPositionDegrees = positionDegrees;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shoulder.setPosition(targetPositionDegrees);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // if(interrupted) {
    //   // stop moving
    //   shoulder.setSpeed(0);
    // }
    //otherwise, do nothing... i.e. keep holding last commanded position on exit
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(targetPositionDegrees - m_shoulder.getPosition()) < Constants.SetpointTolerances.SHOULDER_ANGLE_TOLERANCE;
  }
}
