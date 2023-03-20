// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.waist;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Waist;

public class ZeroWaistPosition extends CommandBase {
  private Waist m_waist;

  private boolean m_reachedMagnetStart = false;

  private double m_magnetRangeStartDegrees;
  private double m_magnetRangeEndDegrees;

  private boolean m_finished = false;

  /** Creates a new ZeroWaistPosition. */
  public ZeroWaistPosition() {
    m_waist = Waist.getInstance();
    addRequirements(m_waist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_waist.setEncoderPosition(0.0);

    // Member variables must be reset during init because the command object instance is sometimes reused,
    // for example when the command is bound to a button
    m_reachedMagnetStart = false;
    m_finished = false;
  }

  @Override
  public void execute() {
    // Find boundaries of magnet
    if (!m_finished) {
      m_waist.setSpeed(0.2);

      if (m_waist.getZeroSensor() && !m_reachedMagnetStart) {
        m_magnetRangeStartDegrees = m_waist.getPosition();
        SmartDashboard.putNumber("Magnet Start", m_magnetRangeStartDegrees);
        m_reachedMagnetStart = true;
      }
      else if (m_reachedMagnetStart && !m_waist.getZeroSensor()) {
        m_waist.setSpeed(0.0);
        m_magnetRangeEndDegrees = m_waist.getPosition();
        m_waist.setSpeed(0.0);
        m_finished = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      double sensorPositionDegrees = (m_magnetRangeEndDegrees + m_magnetRangeStartDegrees) / 2.0;
      double degreesPastSensor = m_waist.getPosition() - sensorPositionDegrees;
      m_waist.setEncoderPosition(Constants.ZeroingOffsets.WAIST_ZERO_SENSOR_OFFSET + degreesPastSensor);
    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
