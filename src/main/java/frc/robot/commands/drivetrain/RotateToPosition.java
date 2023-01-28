// Commands the robot to move a specified distance in inches

package frc.robot.commands.drivetrain;

import frc.robot.subsystems.Drivetrain;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import java.lang.Math;

public class RotateToPosition extends CommandBase implements Loggable {
    private Drivetrain m_driveTrain;
    private double m_degrees;
    private double m_acceptableErrorDegrees = 1;
    private PIDController m_pidController = new PIDController(0.0125, 0, 0);
    private boolean wasWithinTolerance = false;

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
        // m_driveTrain.resetGyro();
    }

    // Returns true when we are within an acceptable distance of our target position
    @Override
    public boolean isFinished() {
        double angleError = getAngleError();
        boolean withinTolerance = Math.abs(angleError) < m_acceptableErrorDegrees;

        if (withinTolerance && wasWithinTolerance) {
            return true;
        } else {
            wasWithinTolerance = withinTolerance;
            return false;
        }
    }

    @Log
    private double getAngleError() {
        return m_degrees - m_driveTrain.getYaw();
    }

    @Log
    private double getRotationPower() {
        double rotationPower = m_pidController.calculate(m_driveTrain.getYaw(), m_degrees);
        rotationPower = MathUtil.clamp(rotationPower, -0.5, 0.5);

        System.out.println("Error: " + getAngleError() + ", Power: " + rotationPower + ", Yaw: " + m_driveTrain.getYaw());
        return rotationPower; 
    }

    @Log
    private double getYaw() {
        return m_driveTrain.getYaw();
    }
    

}
