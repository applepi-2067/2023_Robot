// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDeviceIDs;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants.CANDeviceIDs;
import frc.robot.utils.Util;

public class ClawBelt extends SubsystemBase implements Loggable {

  private static ClawBelt instance = null;
  private static final boolean INVERT_MOTOR = false;
  private final TalonSRX m_motor;

  public static ClawBelt getInstance() {
    if (instance == null) {
      instance = new ClawBelt();
    }
    return instance;
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
  
  /** Creates a new ClawBelt. */
  private ClawBelt() {
    m_motor = new TalonSRX(CANDeviceIDs.MOTOR_CLAWBELT_ID);
    m_motor.setInverted(INVERT_MOTOR);
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