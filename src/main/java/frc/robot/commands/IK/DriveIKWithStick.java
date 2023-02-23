// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IK;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.arm.SetArmExtension;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Waist;
import frc.robot.utils.InverseKinematics;

public class DriveIKWithStick extends CommandBase {
  private Arm m_arm = Arm.getInstance();
  private Waist m_waist = Waist.getInstance();
  private Shoulder m_shoulder = Shoulder.getInstance();
  private double m_xSetpoint = Constants.IKOffsets.MINIMUM_ARM_LENGTH;
  private double m_ySetpoint = 0.0;
  private double m_zSetpoint = 0.0; // Init zero which is level with shoulder axis

  private XboxController m_controller;
  private double STICK_SCALE_FACTOR = 100.0;

  /** Creates a new RobotRelativeIK. */
  public DriveIKWithStick(XboxController controller) {
    addRequirements(m_arm);
    addRequirements(m_shoulder);
    addRequirements(m_waist);

    m_controller = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get the stick values and update the latent x, y, z setpoint
    m_xSetpoint -= m_controller.getRightY() / STICK_SCALE_FACTOR;
    m_ySetpoint -= m_controller.getRightX() / STICK_SCALE_FACTOR;
    m_zSetpoint -= m_controller.getLeftY() / STICK_SCALE_FACTOR;

    // bounds checks
    // This is where my bounds checks would go... IF I HAD ANY

    double shoulderAngleDegrees = InverseKinematics.getThetaShoulder(m_xSetpoint, m_ySetpoint, m_zSetpoint);
    double waistAngleDegrees = InverseKinematics.getThetaWaist(m_xSetpoint, m_ySetpoint, m_zSetpoint);
    double armLength = InverseKinematics.getArmLength(m_xSetpoint, m_ySetpoint, m_zSetpoint);  // Total length of arm from axis of shoulder

    
    // Calculate length the arm needs to extend
    armLength -= Constants.IKOffsets.MINIMUM_ARM_LENGTH;
    armLength = Math.max(armLength, 0);
    
    m_arm.setPosition(armLength);
    m_shoulder.setPosition(shoulderAngleDegrees);
    m_waist.setPosition(waistAngleDegrees);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
