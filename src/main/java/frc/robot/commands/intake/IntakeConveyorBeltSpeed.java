// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeConveyorBelt;

public class IntakeConveyorBeltSpeed extends CommandBase {
  private static IntakeConveyorBelt m_IntakeConveyorBelt;
  Boolean RollersOn1;

  public IntakeConveyorBeltSpeed(Boolean RollersOn) {
    m_IntakeConveyorBelt = IntakeConveyorBelt.getInstance();
    addRequirements(m_IntakeConveyorBelt);
    RollersOn1 = RollersOn;

  }

  @Override
  public void initialize() {}
  
  @Override
  public void execute() {
    if (RollersOn1 == true) {
      m_IntakeConveyorBelt.setSpeed(0.3);
    }
    else{
      m_IntakeConveyorBelt.setSpeed(0.0);}
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
