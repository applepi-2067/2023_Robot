// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANDeviceIDs;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.utils.Util;

public class ClawBelt extends SubsystemBase implements Loggable {

  private static ClawBelt instance = null;
  private static TalonSRX m_motor;
  private static DigitalInput m_gamePieceSensor;

  private static final boolean INVERT_MOTOR = false;

  // Current limit configuration
  private SupplyCurrentLimitConfiguration talonCurrentLimit;
  private final boolean ENABLE_CURRENT_LIMIT = true;
  private final double CONTINUOUS_CURRENT_LIMIT = 10;// amps
  private final double TRIGGER_THRESHOLD_LIMIT = 20; // amp
  private final double TRIGGER_THRESHOLD_TIME = 0.3; // s

  public static ClawBelt getInstance() {
    if (instance == null) {
      instance = new ClawBelt();
    }
    return instance;
  }

  /** Creates a new ClawBelt. */
  private ClawBelt() {
    m_motor = new TalonSRX(CANDeviceIDs.CLAWBELT_MOTOR_ID);
    m_motor.setInverted(INVERT_MOTOR);
    m_motor.setNeutralMode(NeutralMode.Coast);

    talonCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT,
      CONTINUOUS_CURRENT_LIMIT, TRIGGER_THRESHOLD_LIMIT, TRIGGER_THRESHOLD_TIME);
    m_motor.configSupplyCurrentLimit(talonCurrentLimit);

    m_gamePieceSensor = new DigitalInput(Constants.DiscreteInputs.CLAW_IR_SENSOR_DI);
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

  @Log
  public boolean isGamePieceInClaw() {
    return !m_gamePieceSensor.get();
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}