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
    private Drivetrain m_driveTrain;
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

    private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(200, 80);
    // private ProfiledPIDController m_pidController = new ProfiledPIDController(0.016, 0.08, 0.001, constraints);
    private ProfiledPIDController m_pidController = new ProfiledPIDController(0.009, 0.0, 0.0, constraints);


    public RotateToPosition(Drivetrain driveTrain, double degrees) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveTrain);
        m_driveTrain = driveTrain;
        m_degrees = degrees;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_driveTrain.resetGyro();
        m_pidController.reset(0);

        // Sets a limit on integrator range, allowing for more aggressive integrator accumulation via high Ki
        // without runaway accumulation. Set empirically by setting Kp to zero and Ki to a small value, then
        // increasing maximumIntegral until the max integral term is large enough to move to the set point on its own
        // m_pidController.setIntegratorRange(-0.1, 0.1);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_driveTrain.arcadeDrive(0, getRotationPower());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_driveTrain.arcadeDrive(0, 0);
        m_driveTrain.resetGyro();
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
        return m_degrees - m_driveTrain.getYaw();
    }

    @Log
    private double getRotationPower() {
        double rotationPower = m_pidController.calculate(m_driveTrain.getYaw(), m_degrees);

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
