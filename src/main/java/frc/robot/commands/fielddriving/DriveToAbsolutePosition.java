// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.fielddriving;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DriveToAbsolutePosition extends CommandBase {
  private final Drivetrain m_drivetrain = Drivetrain.getInstance();
  private Pose2d m_absoluteDestinationPose;
  private double m_destinationDistance;
  
  private ProfiledPIDController m_distanceController;
  private PIDController m_rotationController;

  /** Creates a new DriveToTargetOffset. */
  public DriveToAbsolutePosition(Pose2d absoluteDestinationPose) {
    addRequirements(m_drivetrain);
    m_absoluteDestinationPose = absoluteDestinationPose;
    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(Constants.Drivetrain.MAX_DRIVETRAIN_VELOCITY, Constants.Drivetrain.MOTOR_ACCELERATION);
    m_distanceController = new ProfiledPIDController(1.5, 0.0, 0.0, constraints);
    m_distanceController.setGoal(0);
    m_rotationController = new PIDController(1.3, 0, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_destinationDistance = getDistanceFromDestination();
    m_distanceController.setGoal(m_destinationDistance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d latestRobotPose = m_drivetrain.getLatestRobotPose2d();

    // Calculate wheel velocities to drive toward target
    double distanceTraveled = m_destinationDistance - getDistanceFromDestination();
    double distanceControlOutput = m_distanceController.calculate(distanceTraveled);
    double leftTrackSpeedDistance = distanceControlOutput;
    double rightTrackSpeedDistance = distanceControlOutput;

    // Calculate wheel velocities to turn to heading
    m_rotationController.setSetpoint(getDestinationAbsoluteHeading());
    double rotationControlOutput = m_rotationController.calculate(latestRobotPose.getRotation().getRadians());
    double leftTrackSpeedRotation = -rotationControlOutput;
    double rightTrackSpeedRotation = rotationControlOutput;

    double leftTrackSpeed = clampVelocity(leftTrackSpeedDistance + leftTrackSpeedRotation);
    double rightTrackSpeed = clampVelocity(rightTrackSpeedDistance + rightTrackSpeedRotation);

    m_drivetrain.setSetPointVelocity(leftTrackSpeed, rightTrackSpeed);
    SmartDashboard.putNumber("Left Wheel Speed", leftTrackSpeed);
    SmartDashboard.putNumber("Right Wheel Speed", rightTrackSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setSetPointVelocity(0.0, 0.0);
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

  private double getDestinationAbsoluteHeading() {
    Pose2d latestRobotPose = m_drivetrain.getLatestRobotPose2d();
    double xError = m_absoluteDestinationPose.getX() - latestRobotPose.getX();
    double yError = m_absoluteDestinationPose.getY() - latestRobotPose.getY();
    return Math.atan2(yError, xError);
  }

  private double clampVelocity(double input) {
    return MathUtil.clamp(input, -Constants.Drivetrain.MAX_DRIVETRAIN_VELOCITY, Constants.Drivetrain.MAX_DRIVETRAIN_VELOCITY);
  }
}