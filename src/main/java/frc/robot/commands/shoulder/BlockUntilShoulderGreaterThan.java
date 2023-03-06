package frc.robot.commands.shoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shoulder;

public class BlockUntilShoulderGreaterThan extends CommandBase {

  private Shoulder m_shoulder;
  private double m_positionThresholdDegrees = 0.0;

  /**
   * 
   * @param positionMeters Command ends when arm is less than this length
   */
  public BlockUntilShoulderGreaterThan(double positionDegrees) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shoulder = Shoulder.getInstance();
    addRequirements(m_shoulder);

    m_positionThresholdDegrees = positionDegrees;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_shoulder.getPosition() > m_positionThresholdDegrees;
  }
}