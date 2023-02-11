package frc.robot.commands.intake;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeConveyor;

public class IntakeConveyorOut extends CommandBase {
    private static IntakeConveyor m_IntakeConveyor;

    public IntakeConveyorOut() {
        m_IntakeConveyor = IntakeConveyor.getInstance();
        addRequirements(m_IntakeConveyor);
    
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute() { 
        m_IntakeConveyor.out();
    }

    @Override
    public void end(boolean interrupted) {
  }
  
  @Override
  public boolean isFinished() {
      return true;
  }
}
