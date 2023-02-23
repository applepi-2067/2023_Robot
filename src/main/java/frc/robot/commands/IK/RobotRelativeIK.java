// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IK;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.arm.SetArmExtension;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Waist;
import frc.robot.utils.InverseKinematics;

public class RobotRelativeIK extends CommandBase {
  private Arm m_arm = Arm.getInstance();
  private Waist m_waist = Waist.getInstance();
  private Shoulder m_shoulder = Shoulder.getInstance();
  private double m_armLength;
  private double m_shoulderAngleDegrees;
  private double m_waistAngleDegrees;
  /** Creates a new RobotRelativeIK. */
  public RobotRelativeIK(double x, double y, double z) {
    // bounds checks
    // Enforce minimum Z height
    z = Math.max(z, Constants.IKConstraints.MINIMUM_Z_HEIGHT);

    // Convert from Z relative to floor to Z relative to shoulder
    z = z - Constants.IKOffsets.SHOULDER_HEIGHT;

    addRequirements(m_arm);
    addRequirements(m_shoulder);
    addRequirements(m_waist);

    m_shoulderAngleDegrees = InverseKinematics.getThetaShoulder(x, y, z);
    m_waistAngleDegrees = InverseKinematics.getThetaWaist(x, y, z);
    m_armLength = InverseKinematics.getArmLength(x, y, z);  // Total length of arm from axis of shoulder

    // Calculate length the arm needs to extend
    m_armLength -= Constants.IKOffsets.MINIMUM_ARM_LENGTH;
    m_armLength = Math.max(m_armLength, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.setPosition(m_armLength);
    m_shoulder.setPosition(m_shoulderAngleDegrees);
    m_waist.setPosition(m_waistAngleDegrees);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_shoulderAngleDegrees - m_shoulder.getPosition()) < Constants.SetpointTolerances.SHOULDER_ANGLE_TOLERANCE && 
    Math.abs(m_armLength - m_arm.getPosition()) < Constants.SetpointTolerances.ARM_METERS_TOLERANCE && 
    Math.abs(m_waistAngleDegrees - m_waist.getPosition()) < Constants.SetpointTolerances.WAIST_ANGLE_TOLERANCE;
  }
}
