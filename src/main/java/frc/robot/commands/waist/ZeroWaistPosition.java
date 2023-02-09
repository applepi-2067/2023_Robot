// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.waist;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Waist;

public class ZeroWaistPosition extends CommandBase {
  private Waist m_waist;

  private boolean reachedMagnetStart = false;
  private boolean reachedMagnetEnd = false;

  private double magnetRangeStart;
  private double magnetRangeEnd;

  /** Creates a new ZeroWaistPosition. */
  public ZeroWaistPosition() {
    m_waist = Waist.getInstance();
    addRequirements(m_waist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_waist.setEncoderPosition(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_waist.setSpeed(0.05);

    if (m_waist.getZeroSensor() && !reachedMagnetStart) {
      magnetRangeStart = m_waist.getPosition();
      SmartDashboard.putNumber("Magnet Start", magnetRangeStart);
      reachedMagnetStart = true;
    }
    else if (reachedMagnetStart && !m_waist.getZeroSensor()) {
      m_waist.setSpeed(0.0);
      magnetRangeEnd = m_waist.getPosition();
      SmartDashboard.putNumber("Magnet End", magnetRangeEnd);
      reachedMagnetEnd = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    double sensorPosition = (magnetRangeEnd - magnetRangeStart) / 2.0;
    m_waist.setEncoderPosition(sensorPosition);
    //m_waist.setPosition(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return reachedMagnetEnd;
  }
}
