package frc.robot.commands.estop;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Make the drivetrain stop by setting wheel velocity setpoint to zero
*/
public class StopDrivetrain extends CommandBase {
    private static Drivetrain m_drivetrain;

    public StopDrivetrain(double meters) { 
        Drivetrain drivetrain = Drivetrain.getInstance(); 
        addRequirements(drivetrain);
        m_drivetrain = drivetrain;
    }

    @Override
    public void initialize() {}


    @Override
    public void execute() {
        m_drivetrain.setSetPointVelocity(0.0, 0.0)
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}