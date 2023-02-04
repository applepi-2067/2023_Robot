package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawBelt;

public class GrabPiece extends CommandBase {
    private static ClawBelt m_clawBelt;

    public GrabPiece(Boolean spin) {
        // Use addRequirements() here to declare subsystem dependencies.
        ClawBelt clawbelt = ClawBelt.getInstance();
        addRequirements(clawbelt);
        m_clawBelt = clawbelt;
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
