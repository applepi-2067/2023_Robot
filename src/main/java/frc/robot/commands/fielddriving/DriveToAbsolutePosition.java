// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.fielddriving;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DriveToAbsolutePosition extends CommandBase {
  private final Drivetrain m_drivetrain = Drivetrain.getInstance();
  private Pose2d m_absoluteDestinationPose;
  private double m_destinationDistance;
  private boolean m_driveBackwards;
  
  private ProfiledPIDController m_distanceController;
  private PIDController m_rotationController;

  private final double MAX_VELOCITY = 7;  // m/s
  private final double MAX_ACCELERATION = 8;  // m/s^2

  public DriveToAbsolutePosition(Pose2d absoluteDestinationPose, double velocityScaling, boolean driveBackwards) {
    addRequirements(m_drivetrain);
    m_absoluteDestinationPose = absoluteDestinationPose;
    m_driveBackwards = driveBackwards;
    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(MAX_VELOCITY * velocityScaling, MAX_ACCELERATION);
    m_distanceController = new ProfiledPIDController(2.0, 0.0, 0.0, constraints);
    m_rotationController = new PIDController(1.3, 0, 0);
  }

  public DriveToAbsolutePosition(Pose2d absoluteDestinationPose) {
    this(absoluteDestinationPose, 1.0, false);
  }

  public DriveToAbsolutePosition(Pose2d absoluteDestinationPose, boolean driveBackwards) {
    this(absoluteDestinationPose, 1.0, driveBackwards);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_destinationDistance = getDistanceFromDestination();
    m_distanceController.setGoal(m_destinationDistance);
    m_rotationController.setSetpoint(0.0);

    // Reset controller errors since this object can be called multiple times
    m_distanceController.reset(0.0);
    m_rotationController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Calculate wheel velocities to drive toward target
    double distanceTraveled = m_destinationDistance - getDistanceFromDestination();
    double distanceControlOutput = m_distanceController.calculate(distanceTraveled);
    if (m_driveBackwards) {
      distanceControlOutput *= -1.0;
    }
    double leftTrackSpeedDistance = distanceControlOutput;
    double rightTrackSpeedDistance = distanceControlOutput;

    // Calculate wheel velocities to turn to heading
    double rotationControlOutput = m_rotationController.calculate(getHeadingErrorRadians());
    double leftTrackSpeedRotation = -rotationControlOutput;
    double rightTrackSpeedRotation = rotationControlOutput;

    double leftTrackSpeed = clampVelocity(leftTrackSpeedDistance + leftTrackSpeedRotation);
    double rightTrackSpeed = clampVelocity(rightTrackSpeedDistance + rightTrackSpeedRotation);

    m_drivetrain.setSetPointVelocity(leftTrackSpeed, rightTrackSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean withinPositionTolerance = Math.abs(getDistanceFromDestination()) < Constants.SetpointTolerances.AUTO_DISTANCE_TOLERANCE;
    boolean withinVelocityTolerance = Math.abs(m_distanceController.getVelocityError()) < Constants.SetpointTolerances.AUTO_VELOCITY_TOLERANCE;
    return withinPositionTolerance && withinVelocityTolerance;
  }

  private double getDistanceFromDestination() {
    Pose2d latestRobotPose = m_drivetrain.getLatestRobotPose2d();
    double xError = m_absoluteDestinationPose.getX() - latestRobotPose.getX();
    double yError = m_absoluteDestinationPose.getY() - latestRobotPose.getY();
    return Math.sqrt(xError * xError + yError * yError);
  }

  private double getHeadingErrorRadians() {
    Pose2d latestRobotPose = m_drivetrain.getLatestRobotPose2d();
    double xError = m_absoluteDestinationPose.getX() - latestRobotPose.getX();
    double yError = m_absoluteDestinationPose.getY() - latestRobotPose.getY();
    
    Rotation2d desiredHeading = new Rotation2d(Math.atan2(yError, xError));
    if (m_driveBackwards) {
      desiredHeading = desiredHeading.minus(new Rotation2d(Math.PI));
    }

    Rotation2d currentHeading = latestRobotPose.getRotation();

    double headingError = currentHeading.minus(desiredHeading).getRadians();
    return headingError;
  }

  private double clampVelocity(double input) {
    return MathUtil.clamp(input, -MAX_VELOCITY, MAX_VELOCITY);
  }
}
