// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeConveyorBelt;

public class IntakeConveyorBeltSpeed extends CommandBase {
  private static IntakeConveyorBelt m_intakeConveyorBelt;
  double m_rollerSpeed;

  public IntakeConveyorBeltSpeed(double rollerSpeed) {
    m_intakeConveyorBelt = IntakeConveyorBelt.getInstance();
    addRequirements(m_intakeConveyorBelt);
    m_rollerSpeed = rollerSpeed;

  }

  @Override
  public void initialize() {}
  
  @Override
  public void execute() {
      m_intakeConveyorBelt.setSpeed(m_rollerSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted)
    {m_intakeConveyorBelt.setSpeed(0.0);}
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
