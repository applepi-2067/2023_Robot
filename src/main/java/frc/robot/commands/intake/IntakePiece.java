package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakePiece extends CommandBase {
    private static Intake m_intake;

    public IntakePiece(Boolean on) {
        // Use addRequirements() here to declare subsystem dependencies.
        Intake intake = Intake.getInstance();
        addRequirements(intake);
        m_intake = intake;
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
