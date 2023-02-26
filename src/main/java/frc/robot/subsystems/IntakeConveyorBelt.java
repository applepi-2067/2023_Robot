package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants.CANDeviceIDs;
import frc.robot.utils.Util;

public class IntakeConveyorBelt extends SubsystemBase implements Loggable {

  private static IntakeConveyorBelt instance = null;

  private final TalonSRX m_motor;
  private static final boolean INVERT_MOTOR = false;

    // Current limit configuration
    private SupplyCurrentLimitConfiguration talonCurrentLimit;
    private final boolean ENABLE_CURRENT_LIMIT = true;
    private final double CONTINUOUS_CURRENT_LIMIT = 20; // amps
    private final double TRIGGER_THRESHOLD_LIMIT = 30; // amp
    private final double TRIGGER_THRESHOLD_TIME = 0.5; // s

  public static IntakeConveyorBelt getInstance() {
    if (instance == null) {
      instance = new IntakeConveyorBelt();
    }
    return instance;
  }

  private IntakeConveyorBelt() {
    m_motor = new TalonSRX(CANDeviceIDs.INTAKE_CONVEYOR_MOTOR_ID);
    m_motor.setInverted(INVERT_MOTOR);

    talonCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT,
      CONTINUOUS_CURRENT_LIMIT, TRIGGER_THRESHOLD_LIMIT, TRIGGER_THRESHOLD_TIME);
    m_motor.configSupplyCurrentLimit(talonCurrentLimit);
  }

  /**
   * Set motor speed.
   * 
   * @param speed (-1.0 to 1.0)
   */
  public void setSpeed(double speed) {
    speed = Util.limit(speed);
    m_motor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
