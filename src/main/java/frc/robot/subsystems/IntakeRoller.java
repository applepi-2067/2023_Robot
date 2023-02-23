// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import frc.robot.utils.Util;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants;

public class IntakeRoller extends SubsystemBase implements Loggable {

  private static IntakeRoller instance = null;
  private final TalonFX m_leftMotor = new TalonFX(Constants.CANDeviceIDs.INTAKE_LEFT_ROLLER_MOTOR_ID);
  private final TalonFX m_rightMotor = new TalonFX(Constants.CANDeviceIDs.INTAKE_RIGHT_ROLLER_MOTOR_ID);

  // Current limit configuration
  private SupplyCurrentLimitConfiguration talonCurrentLimit;
  private final boolean ENABLE_CURRENT_LIMIT = true;
  private final double CONTINUOUS_CURRENT_LIMIT = 15; // amps
  private final double TRIGGER_THRESHOLD_LIMIT = 30; // amp
  private final double TRIGGER_THRESHOLD_TIME = 0.2; // s

  public static IntakeRoller getInstance() {
    if (instance == null) {
      instance = new IntakeRoller();
    }
    return instance;
  }

  /** Creates a new Intake. */
  private IntakeRoller() {
    m_leftMotor.configFactoryDefault();
    m_rightMotor.configFactoryDefault();
    m_leftMotor.setInverted(false);
    m_rightMotor.setInverted(true);
    talonCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT,
        CONTINUOUS_CURRENT_LIMIT, TRIGGER_THRESHOLD_LIMIT, TRIGGER_THRESHOLD_TIME);

    m_rightMotor.configSupplyCurrentLimit(talonCurrentLimit);
    m_leftMotor.configSupplyCurrentLimit(talonCurrentLimit);

  }

  /**
   * Set motor speed.
   * 
   * @param speed (-1.0 to 1.0)
   */
  public void setSpeed(double speed) {
    speed = Util.limit(speed);
    m_leftMotor.set(TalonFXControlMode.PercentOutput, speed);
    m_rightMotor.set(TalonFXControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {

  }

}