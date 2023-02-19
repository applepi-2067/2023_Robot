package frc.robot.commands.intake;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeConveyorExtension;

public class IntakeConveyorIn extends CommandBase {
    enum ConveyorDirection {
        RETRACTED,
        EXTENDED
      }
    private static IntakeConveyorExtension m_IntakeConveyor;
    boolean m_conveyorDirection;
    public IntakeConveyorIn(boolean conveyorDirection) {
        m_IntakeConveyor = IntakeConveyorExtension.getInstance();
        addRequirements(m_IntakeConveyor);
        m_conveyorDirection = conveyorDirection; 
    
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute() { 
        if (m_conveyorDirection){
            m_IntakeConveyor.in();
        }
        else{
            m_IntakeConveyor.out();
        }
    }

    @Override
    public void end(boolean interrupted) {
  }
  
  @Override
  public boolean isFinished() {
      return true;
  }
}
