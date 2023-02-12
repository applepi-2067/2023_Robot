package frc.robot.commands.intake;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeConveyor;

public class IntakeConveyorIn extends CommandBase {
    private static IntakeConveyor m_IntakeConveyor;
    Boolean ActivateConveyorIn;
    public IntakeConveyorIn(Boolean ConveyorIn) {
        m_IntakeConveyor = IntakeConveyor.getInstance();
        addRequirements(m_IntakeConveyor);
        ActivateConveyorIn = ConveyorIn; 
    
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute() { 
        if (ActivateConveyorIn == true){
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
