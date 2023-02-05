package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawGrasp;

public class ClawGraspCommand extends CommandBase {
    private static ClawGrasp m_clawSolenoid;

    public ClawGraspCommand(Boolean open) {
        // Use addRequirements() here to declare subsystem dependencies.
        ClawGrasp ClawGraspCommand = ClawGrasp.getInstance();
        addRequirements(ClawGraspCommand);
        m_clawSolenoid = ClawGraspCommand;
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

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
