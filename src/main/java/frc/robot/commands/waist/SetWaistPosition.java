// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.waist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Waist;

public class SetWaistPosition extends CommandBase {

  private Waist m_waist;
  private double targetPositionDegrees = 0.0;

  /** Creates a new SetWaistPosition. */
  public SetWaistPosition(double positionDegrees) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_waist = Waist.getInstance();
    addRequirements(m_waist);

    targetPositionDegrees = positionDegrees;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_waist.setPosition(targetPositionDegrees);
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
    return Math.abs(targetPositionDegrees - m_waist.getPosition()) < Constants.SetpointTolerances.WAIST_ANGLE_TOLERANCE;
  }
}
