// Commands the robot to move a specified distance in inches

package frc.robot.commands.drivetrain;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.Math;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class RotateToAbsolutePosition extends CommandBase implements Loggable {
    private Drivetrain m_drivetrain;
    private double m_degreesAbsolute;
    private final double ANGLE_TOLERANCE = 1; // deg
    private final double ANGULAR_VELOCITY_TOLERANCE = 5;  // deg/s
    private final double MINIMUM_POWER = 0.20; // Minimum power to turn the robot at all

    // Velocity and acceleration constrained PID control. maxVelocity and maxAcceleration are deg/s and deg/s^2, respectively
    // Ex. a maxAcceleration of 10 deg/s^2 would reach a 40 deg/s angular velocity in 4 seconds
    // Reference values for maximum velocity on 2022 robot:
    // 50 - pokey, motor power = 33%
    // 100 - decent, motor power = 42%
    // 150 - fast, motor power = 50%
    // 200 - competitive, motor power = 61%
    // >250 - scary

    private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(200, 150);
    private ProfiledPIDController m_pidController = new ProfiledPIDController(0.007, 0.0, 0.0, constraints);


    public RotateToAbsolutePosition(double degreesAbsolute) { 
        Drivetrain drivetrain = Drivetrain.getInstance(); 
        addRequirements(drivetrain);
        m_drivetrain = drivetrain;
        m_degreesAbsolute = shiftAngleHalfCircle(degreesAbsolute);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_pidController.reset(0);
        m_pidController.setGoal(m_degreesAbsolute);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_drivetrain.arcadeDrive(0, getRotationPower());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.arcadeDrive(0, 0);
    }

    // Returns true when we are within an acceptable distance of our target position
    @Override
    public boolean isFinished() {
        double angleError = getAngleError();
        boolean withinPositionTolerance = Math.abs(angleError) < ANGLE_TOLERANCE;
        boolean withinVelocityTolerance = Math.abs(m_pidController.getVelocityError()) < ANGULAR_VELOCITY_TOLERANCE;
        return withinPositionTolerance && withinVelocityTolerance;
    }

    /**
     * Return the angle error
     */
    @Log
    private double getAngleError() {
        return m_degreesAbsolute - getCurrentAngleDegrees();
    }

    /**
     * Return the current absolute angle [-180, 180]
     */
    private double getCurrentAngleDegrees() {
        double currentAngle = m_drivetrain.getLatestRobotPose2d().getRotation().getDegrees();
        return shiftAngleHalfCircle(currentAngle);
    }

    private double shiftAngleHalfCircle(double angleDegrees) {
        if (angleDegrees <= 180) {
            return angleDegrees;
        } else {
            return angleDegrees - 360;
        }
    }

    @Log
    private double getRotationPower() {
        double rotationPower = m_pidController.calculate(getCurrentAngleDegrees());

        // Set "floor" of power output to start at m_minimumPower, the minimum power % to move the robot at all
        if (rotationPower > 0) {
            rotationPower += MINIMUM_POWER;
        } else if (rotationPower < 0) {
            rotationPower -= MINIMUM_POWER;
        }

        // Limit power to 65% no matter what controller asks for
        rotationPower = MathUtil.clamp(rotationPower, -0.65, 0.65);
        return rotationPower; 
    }
}
