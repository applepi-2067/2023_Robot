// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import frc.robot.utils.TalonFXHelper;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.utils.Util;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import frc.robot.Constants;
import frc.robot.Constants.CANDeviceIDs;

public class IntakeV2 extends SubsystemBase implements Loggable {

  private static IntakeV2 instance = null;

  public static IntakeV2 getInstance() {
    if (instance == null) {
      instance = new IntakeV2();
    }

    return instance;
  }

  private final CANSparkMax m_flipMotor = new CANSparkMax(CANDeviceIDs.INTAKE_FLIP_ID, MotorType.kBrushless);
  private final TalonFXHelper m_suckMotor = new TalonFXHelper(Constants.CANDeviceIDs.INTAKE_ROLLER_ID);

  // Current limit configuration
  private SupplyCurrentLimitConfiguration talonCurrentLimit;
  private final boolean ENABLE_CURRENT_LIMIT = true;
  private final double CONTINUOUS_CURRENT_LIMIT = 30; // amps
  private final double TRIGGER_THRESHOLD_LIMIT = 60; // amp
  private final double TRIGGER_THRESHOLD_TIME = 0.5; // s
  private static final int CURRENT_LIMIT = 20; // Amps
  private static double max_voltage_open_loop = 6.0;

  /** Creates a new Intake. */
  private IntakeV2() {
    m_flipMotor.restoreFactoryDefaults();
    m_flipMotor.setSmartCurrentLimit(CURRENT_LIMIT);
    m_flipMotor.setIdleMode(IdleMode.kBrake);
    m_suckMotor.configFactoryDefault();
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
  public void setSuckSpeed(double speed) {
    speed = Util.limit(speed);
    double voltage = Util.limit(speed) * max_voltage_open_loop;
    m_suckMotor.set(TalonFXControlMode.PercentOutput, speed);
    
  }
  public void setFlipSpeed(double speed) {
      speed = Util.limit(speed);
      double voltage = Util.limit(speed) * max_voltage_open_loop;
      m_flipMotor.set(speed);
  }

  public double getMotorCurrent(){
    return m_suckMotor.getStatorCurrent();
  }
    @Override
  public void periodic() {

  }

}