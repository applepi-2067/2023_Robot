package frc.robot.commands.chargestation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class BalanceOnCharge extends CommandBase {
    private static Drivetrain m_driveTrain;

    public BalanceOnCharge() {
        // Use addRequirements() here to declare subsystem dependencies.
        Drivetrain drivetrain = Drivetrain.getInstance();
        addRequirements(drivetrain);
        m_driveTrain = drivetrain;
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double angle = m_driveTrain.getRollDegrees();
        
        if (angleOnIntervalInclusive(angle, -90, -10)) {
            m_driveTrain.arcadeDrive(0.25, 0);
        }
        else if (angleOnIntervalInclusive(angle, -10, -5)) {
            m_driveTrain.arcadeDrive(0.18, 0.0);
        }
        else if (angleOnIntervalInclusive(angle, -5, -2.5)) {
            m_driveTrain.arcadeDrive(0.15, 0.0);
        }
        else if (angleOnIntervalInclusive(angle, -2.5, 2.5)) {
            m_driveTrain.arcadeDrive(0.0, 0.0);
        }
        else if (angleOnIntervalInclusive(angle, 2.5, 5)) {
            m_driveTrain.arcadeDrive(-0.15, 0.0);
        }
        else if (angleOnIntervalInclusive(angle, 5, 10)) {
            m_driveTrain.arcadeDrive(-0.18, 0.0);
        }
        else if (angleOnIntervalInclusive(angle, 10, 90)) {
            m_driveTrain.arcadeDrive(-0.25, 0.0);
        }
    }

    public boolean angleOnIntervalInclusive(double angle, double lowAngle, double highAngle) {
        return ((angle >= lowAngle) && (angle <= highAngle));
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
