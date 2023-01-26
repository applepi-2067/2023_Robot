// Commands the robot to move a specified distance in inches

package frc.robot.commands.drivetrain;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.lang.Math;

public class RotateToPosition extends CommandBase {
    private static Drivetrain m_driveTrain;
    private static double m_degrees;
    private static double m_acceptableErrorDegrees = 1;

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
        m_driveTrain.arcadeDrive(0, 0.2);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_driveTrain.arcadeDrive(0, 0);
    }

    // Returns true when we are within an acceptable distance of our target position
    @Override
    public boolean isFinished() {
        double angleError = getAngleError();
        System.out.println("Angle error = " + angleError);

        return Math.abs(angleError) < m_acceptableErrorDegrees;
    }

    private double getAngleError() {
        return m_degrees - m_driveTrain.getYaw();
    }
}