package frc.robot.commands.estop;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Waist;
import frc.robot.subsystems.Shoulder;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Make arm, waist, and shoulder stop movement by setting their setpoints to their current position
*/
public class StopDrivetrain extends CommandBase {
    private static final Arm m_arm = Arm.getInstance();
    private static final Waist m_waist = Waist.getInstance();
    private static final Shoulder m_shoulder = Shoulder.getInstance();

    public StopDrivetrain() { 
        addRequirements(m_arm);
        addRequirements(m_waist);
        addRequirements(m_shoulder);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_arm.setPosition(m_arm.getPosition());
        m_waist.setPosition(m_waist.getPosition());
        m_shoulder.setPosition(m_shoulder.getPosition());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}