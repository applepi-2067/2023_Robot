// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ZeroArmPosition extends CommandBase {

  private Arm m_arm;

  public ZeroArmPosition() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = Arm.getInstance();
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.setSpeed(-0.6);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setSpeed(0.0);
    m_arm.resetEncoders();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_arm.atEndOfTravel();
  }
}
