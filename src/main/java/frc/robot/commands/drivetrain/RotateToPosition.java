// Commands the robot to move a specified distance in inches

package frc.robot.commands.drivetrain;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.lang.Math;

public class RotateToPosition extends CommandBase {
    private static Drivetrain m_driveTrain;
    private static double m_degrees;
    private static double m_acceptableErrorDegrees = 0.1;

    public RotateToPosition(Drivetrain driveTrain, double degrees) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveTrain);
        m_driveTrain = driveTrain;
        m_degrees = degrees;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_driveTrain.getYaw();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when we are within an acceptable distance of our target position
    @Override
    public boolean isFinished() {
        return false;
    }
}