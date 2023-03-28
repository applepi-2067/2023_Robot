// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.waist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Waist;

public class ZeroWaistPositionCoarse extends CommandBase {
  private Waist m_waist;

  private boolean seekCCW;
  private boolean seekCW;
  private boolean isDone;

  public ZeroWaistPositionCoarse() {
    m_waist = Waist.getInstance();
    addRequirements(m_waist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // There appears to be an issue where even though this should zero the encoder,
    // getPosition() is not guarenteed to read back 0 deg immediately. 
    m_waist.setEncoderPosition(0.0);

    // Member variables must be reset during init because the command object instance is sometimes reused,
    // for example when the command is bound to a button
    seekCCW = true;
    seekCW = false;
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Rapidly seek CCW 180 deg for the magnet
    if (seekCCW && m_waist.getPosition() < 180) {
      m_waist.setSpeed(0.2);
    } else if (seekCCW && m_waist.getPosition() >= 180) {
      seekCCW = false;
      seekCW = true;
    } else if (seekCW && m_waist.getPosition() > -180) {
      m_waist.setSpeed(-0.2);
    } else if (seekCW && m_waist.getPosition() <= -180) {
      seekCCW = false;
      seekCW = false;
    }

    if (m_waist.getZeroSensor()) {
      seekCCW = false;
      seekCW = false;
    }

    // After we find the magnet, move CW until we're just off of it.
    if (!seekCCW && !seekCW && !isDone) {
      m_waist.setSpeed(-0.1);
      if (!m_waist.getZeroSensor()) {
        m_waist.setSpeed(0.0);
        m_waist.setEncoderPosition(0.0);
        isDone = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
