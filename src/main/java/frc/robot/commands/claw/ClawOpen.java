package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawGrasp;

public class ClawOpen extends CommandBase {
    private static ClawGrasp m_claw;

    public ClawOpen() {
        // Use addRequirements() here to declare subsystem dependencies.
        m_claw = ClawGrasp.getInstance();
        addRequirements(m_claw);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() { 
        m_claw.open();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
