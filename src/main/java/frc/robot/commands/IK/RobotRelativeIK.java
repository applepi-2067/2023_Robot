// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IK;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.arm.SetArmExtension;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Waist;

public class RobotRelativeIK extends CommandBase {
  private Arm m_arm = Arm.getInstance();
  private Waist m_waist = Waist.getInstance();
  private Shoulder m_shoulder = Shoulder.getInstance();
  private double m_armLength;
  private double m_shoulderAngle;
  private double m_waistAngle;
  /** Creates a new RobotRelativeIK. */
  public RobotRelativeIK(double x, double y, double z) {
    // Convert from Z relative to floor to Z relative to shoulder
    z = z - Constants.SetpointTolerances.SHOULDER_HEIGHT;

    addRequirements(m_arm);
    addRequirements(m_shoulder);
    addRequirements(m_waist);
    // Use addRequirements() here to declare subsystem dependencies.

    m_armLength = getArmLength(x, y, z);
    m_shoulderAngle = getThetaShoulder(x, y, z);
    m_waistAngle = getThetaWaist(x, y, z);
  }

  private double getArmLength(double x, double y, double z) {
    return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2));
  }

  private double getThetaShoulder(double x, double y, double z) {
    return Math.atan2(z, Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)));
  }

  private double getThetaWaist(double x, double y, double z) {
    return Math.atan2(y, x);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.setPosition(m_armLength);
    m_shoulder.setPosition(m_shoulderAngle);
    m_waist.setPosition(m_waistAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_shoulderAngle - m_shoulder.getPosition()) < Constants.SetpointTolerances.SHOULDER_ANGLE_TOLERANCE && 
    Math.abs(m_armLength - m_arm.getPosition()) < Constants.SetpointTolerances.ARM_METERS_TOLERANCE && 
    Math.abs(m_waistAngle - m_waist.getPosition()) < Constants.SetpointTolerances.WAIST_ANGLE_TOLERANCE;
  }
}
