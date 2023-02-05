// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shoulder;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shoulder;

public class DriveShoulderWithJoystick extends CommandBase {
  private Shoulder shoulder;
  private DoubleSupplier speed;

  public DriveShoulderWithJoystick(DoubleSupplier s) {
    // Use addRequirements() here to declare subsystem dependencies.
    shoulder = Shoulder.getInstance();
    speed = s;

    addRequirements(shoulder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shoulder.setSpeed(speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shoulder.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
