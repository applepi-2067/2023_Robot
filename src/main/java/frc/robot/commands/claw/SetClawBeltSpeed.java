package frc.robot.commands.claw;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawBelt;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Lights.Color;

public class SetClawBeltSpeed extends CommandBase {
    private ClawBelt m_ClawBelt; 
    private Lights m_lights;
    private DoubleSupplier m_ClawBeltSpeed; 

    public SetClawBeltSpeed(DoubleSupplier clawBeltSpeed) {
        m_ClawBelt = ClawBelt.getInstance();
        m_lights = Lights.getInstance();
        addRequirements(m_ClawBelt);
        
        m_ClawBeltSpeed = clawBeltSpeed;
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        if (m_lights.getColor() != Lights.Color.PURPLE) {
            m_ClawBelt.setSpeed(m_ClawBeltSpeed.getAsDouble());
        } else {
            m_ClawBelt.setSpeed(0.0);
        }
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