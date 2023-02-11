// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.utils.Gains;

import java.lang.Math;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Shoulder extends SubsystemBase implements Loggable {

  private static Shoulder instance = null;

  private final CANSparkMax m_motor;
  private final SparkMaxPIDController m_pidController;
  private final RelativeEncoder m_encoder;

  private static final double GEAR_RATIO = (5.0/1.0) * (3.0/1.0) * (66.0 / 22.0) * (54.0 / 16.0);
  private static final double DEGREES_PER_REV = 360.0;
  private static final int CURRENT_LIMIT = 5; //Amps
  private static final boolean INVERT_MOTOR = false;

  // Voltage needed to maintain horizontal arm position.
  private static final double horizontalArbFF = 0.30;

  // PID Coefficients.
  private static Gains gains = new Gains(0.11, 0.001, 0, 0, 1, 0.4);
  private static final int kSlotIdx = 0;

  public static Shoulder getInstance() {
    if (instance == null) {
      instance = new Shoulder();
    }

    return instance;
  }
  
  /** Creates a new Shoulder. */
  private Shoulder() {
    m_motor = new CANSparkMax(Constants.CANDeviceIDs.MOTOR_SHOULDER_ID, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_motor.setSmartCurrentLimit(CURRENT_LIMIT);
    m_motor.setInverted(INVERT_MOTOR);

    m_pidController = m_motor.getPIDController();
    m_encoder = m_motor.getEncoder();

     // Set PID coefficients
     m_pidController.setP(gains.kP);
     m_pidController.setI(gains.kI);
     m_pidController.setD(gains.kD);
     m_pidController.setIZone(gains.kIzone);
     m_pidController.setFF(gains.kF);
     m_pidController.setOutputRange(-gains.kPeakOutput, gains.kPeakOutput);

     if (RobotBase.isSimulation()) {
      REVPhysicsSim.getInstance().addSparkMax(m_motor, DCMotor.getNEO(1));
    }
  }

  private double degreesToMotorRotations(double degrees) {
    return degrees/DEGREES_PER_REV * GEAR_RATIO;
  }

  private double motorRotationsToDegrees(double rotations) {
    return (rotations / GEAR_RATIO) * DEGREES_PER_REV;
  }

  @Log (name = "Motor Current (A)", rowIndex = 2, columnIndex = 3)
  private double getMotorOutputCurrent() {
    return m_motor.getOutputCurrent();
  }

  @Config
  private void setGains(double P, double I, double D, double f, double IZone, double peak) {

  }

  /**
   * Set shoulder rotation.
   * @param degrees
   */
  public void setPosition(double degrees) {
    m_pidController.setReference(
      degreesToMotorRotations(degrees),
      CANSparkMax.ControlType.kPosition,
      kSlotIdx,
      getArbFF()
    );
  }

  /**
   * Get the Arbitrary Feed-Forward term, voltage needed to maintain arm positon. 

   * @return Arbitrary Feed-Forward (Volts)
   */
  public double getArbFF() {
    double degrees = getPosition();
    double radians = Math.toRadians(degrees);

    return Math.cos(radians) * horizontalArbFF;
  }

  /**
   * Set motor speed.
   * @param speed (-1.0 to 1.0)
   */
  public void setSpeed(double speed) {
    final double MAX_VOLTAGE = 2.0;

    m_pidController.setReference(speed * MAX_VOLTAGE, CANSparkMax.ControlType.kVoltage);
  }

  /**
   *  Get shoulder rotation.
   * @return shoulder position in degrees
   */
  @Log (name = "Position (Deg)", rowIndex = 2, columnIndex = 0)
  public double getPosition() {
    return motorRotationsToDegrees(m_encoder.getPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }
}
