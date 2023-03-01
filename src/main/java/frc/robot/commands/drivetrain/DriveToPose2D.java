// Commands the robot to move a specified distance in inches

package frc.robot.commands.drivetrain;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.lang.Math;
import frc.robot.subsystems.Vision;

public class DriveToPose2D extends CommandBase {
    private static Drivetrain m_drivetrain;
    private final double ACCEPTABLE_ERROR_METERS = Units.inchesToMeters(0.1);

    public DriveToPose2D(double meters) { 
        Drivetrain drivetrain = Drivetrain.getInstance(); 
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
        m_drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        m_drivetrain.getLatestRobotPose2d();
    }

    @Override
    public void execute() {
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //might not need something here?
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}