// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class BlockUntilArmLessThan extends CommandBase {

  private Arm m_arm;
  private double m_positionThresholdMeters = 0.0;

  /**
   * 
   * @param positionMeters Command ends when arm is less than this length
   */
  public BlockUntilArmLessThan(double positionMeters) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = Arm.getInstance();
    m_positionThresholdMeters = positionMeters;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_arm.getPosition() < m_positionThresholdMeters;
  }
}
