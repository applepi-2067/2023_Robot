package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawBelt;
import frc.robot.subsystems.ClawGrasp;

public class ClawGrabCancel extends CommandBase {
    private ClawBelt m_clawBelt; 
    private ClawGrasp m_clawGrasp;

    public ClawGrabCancel() {
        m_clawBelt = ClawBelt.getInstance();
        m_clawGrasp = ClawGrasp.getInstance();

        addRequirements(m_clawBelt);
        addRequirements(m_clawGrasp);
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        m_clawBelt.setSpeed(0);
        m_clawGrasp.close();
    }
  
    @Override
    public void end(boolean interrupted) {}
  
    @Override
    public boolean isFinished() {
        return true;
    }
}