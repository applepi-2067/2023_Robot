package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class balanceOnCharge extends CommandBase {
    private static Drivetrain m_driveTrain;

    public balanceOnCharge() {
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
        if (m_driveTrain.getGryoPitch() < -1){
            m_driveTrain.arcadeDrive(-0.2, 0);
        }
        else if (m_driveTrain.getGryoPitch() > 1){
            m_driveTrain.arcadeDrive(0.2, 0);
        }
        else if (m_driveTrain.getGryoPitch() <=1 && m_driveTrain.getGryoPitch() >=-1){
            m_driveTrain.arcadeDrive(0, 0);
        }

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
