// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANDeviceIDs;
import frc.robot.utils.Gains;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import frc.robot.MonkeyTest;

public class Waist extends SubsystemBase implements Loggable {

  private static Waist instance = null;

  private final CANSparkMax m_motor;
  private final SparkMaxPIDController m_pidController;
  private final RelativeEncoder m_encoder;
  private final DigitalInput m_zeroingSensor;

  private static final double GEAR_RATIO = 4.0 * 4.0 * (36.0 / 18.0) * (106.0 / 30.0);
  private static final double DEGREES_PER_REV = 360.0;
  private static final int CURRENT_LIMIT_AMPS = 30;
  private static final boolean INVERT_MOTOR = false;

  private static final int SMART_MOTION_SLOT = 0;

  // PID Coefficients.
  // private Gains gains = new Gains(0.1, 5e-4, 0, 0, 3, 0.7); //raw PI controller gains (non-smart motion)
  private Gains gains = new Gains(3e-4, 3e-6, 0.000156, 0, 1, 0.7); //smart motion gains

  // SmartMotion configs
  private static final double MAX_VELOCITY_RPM = 5_000; // NEO free speed 5676 RPM
  private static final double MIN_VELOCITY_RPM = 0;
  private static final double MAXX_ACCELERATION_RPM_PER_SEC = 8_000;
  private static final double ALLOWED_ERROR = 0.0; //motor rotations

  public static Waist getInstance() {
    if (instance == null) {
      instance = new Waist();
    }

    return instance;
  }
  
  /** Creates a new Waist. */
  private Waist() {
    m_motor = new CANSparkMax(CANDeviceIDs.WAIST_MOTOR_ID, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setSmartCurrentLimit(CURRENT_LIMIT_AMPS);
    m_motor.setInverted(INVERT_MOTOR);

    m_pidController = m_motor.getPIDController();
    m_encoder = m_motor.getEncoder();
    m_zeroingSensor = new DigitalInput(Constants.DiscreteInputs.WAIST_ZEROING_DI);

     // Set PID coefficients
     m_pidController.setP(gains.kP, SMART_MOTION_SLOT);
     m_pidController.setI(gains.kI, SMART_MOTION_SLOT);
     m_pidController.setD(gains.kD, SMART_MOTION_SLOT);
     m_pidController.setIZone(gains.kIzone, SMART_MOTION_SLOT);
     m_pidController.setFF(gains.kF, SMART_MOTION_SLOT);
     m_pidController.setOutputRange(-gains.kPeakOutput, gains.kPeakOutput, SMART_MOTION_SLOT);

     m_pidController.setSmartMotionMaxVelocity(MAX_VELOCITY_RPM, SMART_MOTION_SLOT);
     m_pidController.setSmartMotionMinOutputVelocity(MIN_VELOCITY_RPM, SMART_MOTION_SLOT);
     m_pidController.setSmartMotionMaxAccel(MAXX_ACCELERATION_RPM_PER_SEC, SMART_MOTION_SLOT);
     m_pidController.setSmartMotionAllowedClosedLoopError(ALLOWED_ERROR, SMART_MOTION_SLOT);

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

  public void setEncoderPosition(double encoderPosition) {
    m_encoder.setPosition(degreesToMotorRotations(encoderPosition));
  }

  /**
   * Set waist rotation.
   * @param degrees
   */
  public void setPosition(double degrees) {
    if (MonkeyTest.get() == 6) {
      degrees = 0;
    }
    // m_pidController.setReference(degreesToMotorRotations(degrees), CANSparkMax.ControlType.kPosition);
    m_pidController.setReference(degreesToMotorRotations(degrees), CANSparkMax.ControlType.kSmartMotion, SMART_MOTION_SLOT);
  }

  /**
   * Set motor speed.
   * @param speed (-1.0 to 1.0)
   */
  public void setSpeed(double speed) {
    final double MAX_VOLTAGE = 4.0;

    m_pidController.setReference(speed * MAX_VOLTAGE, CANSparkMax.ControlType.kVoltage);
  }

  /**
   *  Get waist rotation.
   * @return waist position in degrees
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

  @Log (name = "Zero Sensor", rowIndex = 2, columnIndex = 1)
  public boolean getZeroSensor() {
    return !m_zeroingSensor.get();
  }
}
