// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import frc.robot.utils.Util; 
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants;

public class IntakeRoller extends SubsystemBase implements Loggable {
  
  private static IntakeRoller instance = null;
  private final WPI_TalonFX m_leftMotor = new WPI_TalonFX(Constants.CANDeviceIDs.INTAKE_LEFT_ROLLER_MOTOR_ID);
  private final WPI_TalonFX m_rightMotor = new WPI_TalonFX(Constants.CANDeviceIDs.INTAKE_RIGHT_ROLLER_MOTOR_ID);

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
  }
  public void resetEncoders() {
    m_leftMotor.getSensorCollection().setIntegratedSensorPosition(0, Constants.Drivetrain.kTimeoutMs);
    m_rightMotor.getSensorCollection().setIntegratedSensorPosition(0, Constants.Drivetrain.kTimeoutMs);

    m_leftMotor.setSelectedSensorPosition(0.0);
    m_rightMotor.setSelectedSensorPosition(0.0);
  }
            /**
     * Set motor speed.
     * @param speed (-1.0 to 1.0)       
     */
    public void setSpeed(double speed) {
      Util.limit(speed);
      m_leftMotor.set(TalonFXControlMode.PercentOutput, speed);
      m_rightMotor.set(TalonFXControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {

  }

}