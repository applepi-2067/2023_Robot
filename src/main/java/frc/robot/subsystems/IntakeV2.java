// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import frc.robot.utils.TalonFXHelper;
import com.revrobotics.SparkMaxPIDController;
import frc.robot.utils.Util;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import frc.robot.Constants;

public class IntakeV2 extends SubsystemBase implements Loggable {

  private static IntakeV2 instance = null;
  private final SparkMaxPIDController m_flipMotor = new SparkMaxPIDController(Constants.CANDeviceIDs.INTAKE_FLIP_ID);
  private final TalonFXHelper m_suckMotor = new TalonFXHelper(Constants.CANDeviceIDs.INTAKE_ROLLER_ID);

  // Current limit configuration
  private SupplyCurrentLimitConfiguration talonCurrentLimit;
  private final boolean ENABLE_CURRENT_LIMIT = true;
  private final double CONTINUOUS_CURRENT_LIMIT = 30; // amps
  private final double TRIGGER_THRESHOLD_LIMIT = 60; // amp
  private final double TRIGGER_THRESHOLD_TIME = 0.5; // s

  public static IntakeV2 getInstance() {
    if (instance == null) {
      instance = new IntakeV2();
    }
    return instance;
  }

  /** Creates a new Intake. */
  private IntakeV2() {
    m_flipMotor.configFactoryDefault();
    m_suckMotor.configFactoryDefault();

    m_flipMotor.configOpenLoopStatusFrameRates();
    m_suckMotor.configOpenLoopStatusFrameRates();

    talonCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT,
        CONTINUOUS_CURRENT_LIMIT, TRIGGER_THRESHOLD_LIMIT, TRIGGER_THRESHOLD_TIME);

    m_suckMotor.configSupplyCurrentLimit(talonCurrentLimit);
  }

  /**
   * Set motor speed.
   * 
   * @param speed (-1.0 to 1.0)
   */
  public void setSpeed(double speed) {
    speed = Util.limit(speed);
    m_flipMotor.set(TalonFXControlMode.PercentOutput, speed);
    m_suckMotor.set(TalonFXControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {

  }

}