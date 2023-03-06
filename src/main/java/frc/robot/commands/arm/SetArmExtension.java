// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class SetArmExtension extends CommandBase {

  private Arm m_arm;
  private double targetPositionMeters = 0.0;
  // private double positionToleranceMeters = 0.1;

  /** Creates a new SetArmPosition. */
  /**
   * 
   * @param positionMeters the target extension to move the arm to (meters)
   */
  public SetArmExtension(double positionMeters) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = Arm.getInstance();
    targetPositionMeters = positionMeters;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.setPosition(targetPositionMeters);
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
    return Math.abs(targetPositionMeters - m_arm.getPosition()) < Constants.SetpointTolerances.ARM_METERS_TOLERANCE; // never finish - to maintain position after we get to destination
    // return Math.abs(targetPositionMeters - arm.getPosition()) < positionToleranceMeters;
  }
}
