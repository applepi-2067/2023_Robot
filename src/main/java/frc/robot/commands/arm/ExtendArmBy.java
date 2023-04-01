// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ExtendArmBy extends CommandBase {

  private Arm m_arm;
  private double relativePositionMeters = 0.0;

  /** Creates a new SetArmPosition. */
  /**
   * 
   * @param positionMeters the relative amount to extend arm by (meters)
   */
  public ExtendArmBy(double positionMeters) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = Arm.getInstance();
    relativePositionMeters = positionMeters;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.setPosition(m_arm.getPosition() + relativePositionMeters);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted) {
      // stop moving
      m_arm.setSpeed(0);
    }
    //otherwise, do nothing... i.e. keep holding last commanded position on exit
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
