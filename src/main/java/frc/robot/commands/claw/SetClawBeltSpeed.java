package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawBelt;

public class SetClawBeltSpeed extends CommandBase {
    private ClawBelt m_ClawBelt; 
    private double m_ClawBeltSpeed; 

    public SetClawBeltSpeed(double clawBeltSpeed) {
        m_ClawBelt = ClawBelt.getInstance();
        addRequirements(m_ClawBelt);
        
        m_ClawBeltSpeed = clawBeltSpeed;
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        m_ClawBelt.setSpeed(m_ClawBeltSpeed);
    }
  
    @Override
    public void end(boolean interrupted) {
        if (interrupted){
            m_ClawBelt.setSpeed(0.0);
        }
    }
  
    @Override
    public boolean isFinished() {
        return true;
    }
}