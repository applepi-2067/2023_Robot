// Commands the robot to move a specified distance in inches

package frc.robot.commands.drivetrain;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.lang.Math;

public class DriveToPosition extends CommandBase {
    private static Drivetrain m_drivetrain;
    private static double m_meters;
    private static double m_acceptableErrorMeters = Units.inchesToMeters(0.1);

    public DriveToPosition(double meters) { 
        Drivetrain drivetrain = Drivetrain.getInstance(); 
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
        m_drivetrain = drivetrain;
        m_meters = meters;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_drivetrain.resetEncoders();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_drivetrain.setSetPointDistance(m_meters);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Stop the drivetain motors
        m_drivetrain.arcadeDrive(0, 0);
    }

    // Returns true when we are within an acceptable distance of our target position
    @Override
    public boolean isFinished() {
        double metersError = m_meters - m_drivetrain.getDistanceTraveled();
        return (Math.abs(metersError) < m_acceptableErrorMeters);
    }
}