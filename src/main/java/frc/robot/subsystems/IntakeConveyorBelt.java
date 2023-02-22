package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.Constants.CANDeviceIDs;
import frc.robot.utils.Util;

public class IntakeConveyorBelt extends SubsystemBase implements Loggable {

  private static IntakeConveyorBelt instance = null;

  private final TalonSRX m_motor;
  private static final boolean INVERT_MOTOR = false;
  private static final int CURRENT_LIMIT = 10; //Amps

  public static IntakeConveyorBelt getInstance() {
    if (instance == null) {
      instance = new IntakeConveyorBelt();
    }
    return instance;
  }

  private IntakeConveyorBelt() {
    m_motor = new TalonSRX(CANDeviceIDs.INTAKE_CONVEYOR_MOTOR_ID);
    m_motor.setInverted(INVERT_MOTOR);

  }

  /**
   * Set motor speed.
   * 
   * @param speed (-1.0 to 1.0)
   */
  public void setSpeed(double speed) {
    Util.limit(speed);
    m_motor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Config(rowIndex = 0, columnIndex = 5, width = 1, height = 1)
  public void clawIntake(boolean push) {

  }
}
