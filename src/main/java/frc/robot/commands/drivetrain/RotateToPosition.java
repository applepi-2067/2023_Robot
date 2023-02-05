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

public class RotateToPosition extends CommandBase implements Loggable {
    private Drivetrain m_drivetrain;
    private double m_startingDegrees;
    private double m_degrees;
    private final double m_acceptableErrorDegrees = 1;
    private final double m_acceptableErrorDegreesPerSecond = 5;
    private final double m_minimumPower = 0.20; // Minimum power to turn the robot at all

    // Velocity and acceleration constrained PID control. maxVelocity and maxAcceleration are deg/s and deg/s^2, respectively
    // Ex. a maxAcceleration of 10 deg/s^2 would reach a 40 deg/s angular velocity in 4 seconds
    // Reference values for maximum velocity on 2022 robot:
    // 50 - pokey, motor power = 33%
    // 100 - decent, motor power = 42%
    // 150 - fast, motor power = 50%
    // 200 - competitive, motor power = 61%
    // >250 - scary

    private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(200, 150);
    private ProfiledPIDController m_pidController = new ProfiledPIDController(0.014, 0.0, 0.0, constraints);


    public RotateToPosition(double degrees) { 
        Drivetrain drivetrain = Drivetrain.getInstance(); 
        addRequirements(drivetrain);
        m_drivetrain = drivetrain;
        m_degrees = optimizeRotation(degrees);
    }

    private double optimizeRotation(double degrees) {
        degrees = degrees % 360.0;
        if (degrees > 180.0) {
            degrees -= 360.0;
        }
        else if (degrees < -180.0) {
            degrees += 360.0;
        }
        return degrees;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_startingDegrees = m_drivetrain.getYawDegrees();
        m_pidController.reset(0);
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
        boolean withinPositionTolerance = Math.abs(angleError) < m_acceptableErrorDegrees;
        boolean withinVelocityTolerance = Math.abs(m_pidController.getVelocityError()) < m_acceptableErrorDegreesPerSecond;
        return withinPositionTolerance && withinVelocityTolerance;
    }

    @Log
    private double getAngleError() {
        return m_degrees - (m_drivetrain.getYawDegrees() - m_startingDegrees);
    }

    @Log
    private double getRotationPower() {
        double rotationPower = m_pidController.calculate(m_drivetrain.getYawDegrees() - m_startingDegrees, m_degrees);

        // Set "floor" of power output to start at m_minimumPower, the minimum power % to move the robot at all
        if (rotationPower > 0) {
            rotationPower += m_minimumPower;
        } else if (rotationPower < 0) {
            rotationPower -= m_minimumPower;
        }

        // Limit power to 65% no matter what controller asks for
        rotationPower = MathUtil.clamp(rotationPower, -0.65, 0.65);

        // Debug in shuffleboard
        SmartDashboard.putNumber("Position Error", m_pidController.getPositionError());
        SmartDashboard.putNumber("Angle Error", getAngleError());
        SmartDashboard.putNumber("Applied power", rotationPower);
        SmartDashboard.putNumber("Velocity Error", m_pidController.getVelocityError());

        return rotationPower; 
    }
}
