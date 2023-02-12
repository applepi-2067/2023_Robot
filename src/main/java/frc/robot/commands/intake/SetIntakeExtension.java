// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.IntakeExtensionMotor;

public class SetIntakeExtension extends CommandBase {

  private IntakeExtensionMotor intakeextensionmotor;
  private double targetPositionMeters = 0.0;
  // private double positionToleranceMeters = 0.1;

  /** Creates a new SetArmPosition. */
  /**
   * 
   * @param positionMeters the target extension to move the arm to (meters)
   */
  public SetIntakeExtension(double positionMeters) {
    // Use addRequirements() here to declare subsystem dependencies.
    intakeextensionmotor = IntakeExtensionMotor.getInstance();
    addRequirements(intakeextensionmotor);

    targetPositionMeters = positionMeters;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeextensionmotor.setPosition(targetPositionMeters);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted) {
      // stop moving
      intakeextensionmotor.setSpeed(0);
    }
    //otherwise, do nothing... i.e. keep holding last commanded position on exit
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // never finish - to maintain position after we get to destination
    // return Math.abs(targetPositionMeters - arm.getPosition()) < positionToleranceMeters;
  }
}
