// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

import org.apache.commons.lang3.ObjectUtils.Null;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.Constants.CANDeviceIDs;
import frc.robot.commands.intake.SetIntakeExtension;
import frc.robot.utils.Util; 
import frc.robot.utils.Gains;

public class IntakeExtensionMotor extends SubsystemBase {

  public static IntakeExtensionMotor instance = null;

  private final CANSparkMax m_motor;
  private final RelativeEncoder m_encoder;
  private final SparkMaxPIDController m_pidController;

  public static final double GEAR_RATIO = 36.0;  //TODO: set
  public static final double METERS_PER_REV = 1.0; //TODO: set

  public static double max_voltage_open_loop = 1.0;

  // PID Coefficients.
  private Gains gains = new Gains(0.1, 1e-4, 1, 0, 1, 0.4);


  public static IntakeExtensionMotor getInstance() {
    if (instance == null) {
      instance = new IntakeExtensionMotor();
    }
    return instance;
}

      private IntakeExtensionMotor() {
        m_motor = new CANSparkMax(CANDeviceIDs.INTAKE_EXTENSION_MOTOR_ID, MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();
        m_encoder = m_motor.getEncoder();
        m_pidController = m_motor.getPIDController();
    
        // Set PID coefficients
        m_pidController.setP(gains.kP);
        m_pidController.setI(gains.kI);
        m_pidController.setD(gains.kD);
        m_pidController.setIZone(gains.kIzone);
        m_pidController.setFF(gains.kF);
        m_pidController.setOutputRange(-gains.kPeakOutput, gains.kPeakOutput);
   
      }
      
      private double metersToMotorRotations(double degrees) {
        return degrees/METERS_PER_REV * GEAR_RATIO;
      }
    
      private double motorRotationsToMeters(double rotations) {
        return (rotations / GEAR_RATIO) * METERS_PER_REV;
      }

        /**
   * Set motor speed.
   * @param speed (-1.0 to 1.0)
   */
  public void setSpeed(double speed) {
    Util.limit(speed);
    m_pidController.setReference(speed * max_voltage_open_loop, CANSparkMax.ControlType.kVoltage);
  }

  @Config (name = "Output Limit (V)", rowIndex = 2, columnIndex = 2, defaultValueNumeric = 1.0)
  public void setMaxVoltage(double v) {
    max_voltage_open_loop = v;
  }

   /**
   * Set intake position.
   * @param meters
   */
  public void setPosition(double meters) {
    m_pidController.setReference(metersToMotorRotations(meters), CANSparkMax.ControlType.kPosition);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
