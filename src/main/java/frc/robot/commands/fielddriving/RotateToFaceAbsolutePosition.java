package frc.robot.commands.fielddriving;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.lang.Math;

import io.github.oblarg.oblog.annotations.Log;

public class RotateToFaceAbsolutePosition extends CommandBase {
  private Drivetrain m_drivetrain;
  private Rotation2d m_setpointRotationOffset;
  private Pose2d m_absolutePose2d;

  private final double ANGLE_TOLERANCE = 5; // deg
  private final double ANGULAR_VELOCITY_TOLERANCE = 5;  // deg/s
  private final double MINIMUM_POWER = 0.10; // Minimum power to turn the robot at all

  private PIDController m_pidController = new PIDController(0.005, 0.0, 0.0);

  public RotateToFaceAbsolutePosition(Pose2d absolutePose2d) {
    m_absolutePose2d = absolutePose2d;

    m_drivetrain = Drivetrain.getInstance();
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Get latest robot pose.
    Pose2d latestRobotPose2d = m_drivetrain.getLatestRobotPose2d();

    // Calculate angle between poses.
    double xDiff = m_absolutePose2d.getX() - latestRobotPose2d.getX();
    double yDiff = m_absolutePose2d.getY() - latestRobotPose2d.getY();
    m_setpointRotationOffset = Rotation2d.fromRadians(Math.atan2(yDiff, xDiff));

    m_pidController.setSetpoint(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.arcadeDrive(0, getRotationPower());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
  }

  // Returns true when we are within an acceptable distance of our target position
  @Override
  public boolean isFinished() {
    double angleError = getAngleError().getDegrees();
    boolean withinPositionTolerance = Math.abs(angleError) < ANGLE_TOLERANCE;
    boolean withinVelocityTolerance = Math.abs(m_pidController.getVelocityError()) < ANGULAR_VELOCITY_TOLERANCE;
    return withinPositionTolerance && withinVelocityTolerance;
  }

  /**
   * Return the angle error
   */
  @Log
  private Rotation2d getAngleError() {
    Rotation2d angleError = getRobotLatestRotation2d().minus(m_setpointRotationOffset);
    return angleError;
  }

  /**
   * Return the robot's latest rotation
   */
  private Rotation2d getRobotLatestRotation2d() {
    Rotation2d robotLatestRotation2d = m_drivetrain.getLatestRobotPose2d().getRotation();
    return robotLatestRotation2d;
  }

  @Log
  private double getRotationPower() {
    double rotationPower = m_pidController.calculate(getAngleError().getDegrees());

    // Set "floor" of power output to start at m_minimumPower, the minimum power % to move the robot at all
    if (rotationPower > 0) {
      rotationPower += MINIMUM_POWER;
    } 
    else if (rotationPower < 0) {
      rotationPower -= MINIMUM_POWER;
    }

    // Limit power to 40% no matter what controller asks for
    rotationPower = MathUtil.clamp(rotationPower, -0.4, 0.4);
    return rotationPower; 
  }
}
