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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Gains;
import frc.robot.utils.Util;
import frc.robot.Constants;
import frc.robot.Constants.CANDeviceIDs;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Arm extends SubsystemBase implements Loggable {

  private static Arm instance = null;

  private final CANSparkMax m_motor;
  private final SparkMaxPIDController m_pidController;
  private final RelativeEncoder m_encoder;
  private final DigitalInput m_endOfTravelSensor;

  private static final int CURRENT_LIMIT = 10; //Amps
  private static final double GEAR_RATIO = (84 / 29) * (76 / 21);
  private static final double OUTPUT_SPROCKET_PITCH_DIAMETER_METERS = 0.020574;
  private static final double RIGGING_EXTENSION_RATIO = 2.0;
  private static final double METERS_PER_REV = Math.PI * OUTPUT_SPROCKET_PITCH_DIAMETER_METERS * RIGGING_EXTENSION_RATIO;
  private static final boolean INVERT_MOTOR = true;
  public static final double MAX_ARM_EXTENSION_METERS = Units.inchesToMeters(36.25);

  private static double max_voltage_open_loop = 6.0;

  private static final int SMART_MOTION_SLOT = 0;

  // PID Coefficients.
  // private Gains gains = new Gains(0.1, 1e-4, 1, 0, 1, 1.0); //raw PI controller gains (non-smart motion)
  private Gains gains = new Gains(1e-4, 3e-6, 0.000156, 0, 1, 1.0); //smart motion gains
  
  // SmartMotion configs
  private static final double MAX_VELOCITY_RPM = 11_000; // NEO550 free speed 11000RPM
  private static final double MIN_VELOCITY_RPM = 0;
  private static final double MAX_ACCELERATION_RPM_PER_SEC = 30_000;
  private static final double ALLOWED_ERROR = 0.05; //motor rotations

  public static Arm getInstance() {
    if (instance == null) {
      instance = new Arm();
    }

    return instance;
  }

  private Arm() {
    m_endOfTravelSensor = new DigitalInput(Constants.DiscreteInputs.ARM_END_OF_TRAVEL_DI);

    m_motor = new CANSparkMax(CANDeviceIDs.ARM_MOTOR_ID, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_motor.setSmartCurrentLimit(CURRENT_LIMIT);
    m_motor.setInverted(INVERT_MOTOR);
    m_motor.setIdleMode(IdleMode.kBrake);

    m_pidController = m_motor.getPIDController();
    m_encoder = m_motor.getEncoder();

    // Set PID coefficients
    m_pidController.setP(gains.kP, SMART_MOTION_SLOT);
    m_pidController.setI(gains.kI, SMART_MOTION_SLOT);
    m_pidController.setD(gains.kD, SMART_MOTION_SLOT);
    m_pidController.setIZone(gains.kIzone, SMART_MOTION_SLOT);
    m_pidController.setFF(gains.kF, SMART_MOTION_SLOT);
    m_pidController.setOutputRange(-gains.kPeakOutput, gains.kPeakOutput, SMART_MOTION_SLOT);

    m_pidController.setSmartMotionMaxVelocity(MAX_VELOCITY_RPM, SMART_MOTION_SLOT);
    m_pidController.setSmartMotionMinOutputVelocity(MIN_VELOCITY_RPM, SMART_MOTION_SLOT);
    m_pidController.setSmartMotionMaxAccel(MAX_ACCELERATION_RPM_PER_SEC, SMART_MOTION_SLOT);
    m_pidController.setSmartMotionAllowedClosedLoopError(ALLOWED_ERROR, SMART_MOTION_SLOT);

    if (RobotBase.isSimulation()) {
      REVPhysicsSim.getInstance().addSparkMax(m_motor, DCMotor.getNEO(1));
    }
  }

  private double metersToMotorRotations(double meters) {
    return meters/METERS_PER_REV * GEAR_RATIO;
  }

  private double motorRotationsToMeters(double rotations) {
    return (rotations / GEAR_RATIO) * METERS_PER_REV;
  }

  /**
   * Set arm position.
   * @param meters
   */
  public void setPosition(double meters) {
    // Clamp arm extension on [0.0, max extension].
    meters = MathUtil.clamp(meters, 0.0, MAX_ARM_EXTENSION_METERS);
    // m_pidController.setReference(metersToMotorRotations(meters), CANSparkMax.ControlType.kPosition);
    m_pidController.setReference(metersToMotorRotations(meters), CANSparkMax.ControlType.kSmartMotion, SMART_MOTION_SLOT);
  }

  /**
   * Set motor speed.
   * @param speed (-1.0 to 1.0)
   */
  public void setSpeed(double speed) {
    double voltage = Util.limit(speed) * max_voltage_open_loop;
    m_pidController.setReference(voltage, CANSparkMax.ControlType.kVoltage);
  }

  @Log (name = "Velocity (V)", rowIndex = 2, columnIndex = 0)
  double getVelocity() {
    return (m_encoder.getVelocity());
  }
  
  public void resetEncoders() {
    m_encoder.setPosition(0);
  }
  

  /**
   *  Get arm position.
   * @return waist position in meters
   */
  @Log (name = "Position (M)", rowIndex = 2, columnIndex = 0)
  public double getPosition() {
    return motorRotationsToMeters(m_encoder.getPosition());
  }

  @Log (name = "Motor Current (A)", rowIndex = 2, columnIndex = 3)
  private double getMotorOutputCurrent() {
    return m_motor.getOutputCurrent();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }

  @Log
  public boolean atEndOfTravel() {
    return ! m_endOfTravelSensor.get();
  }
}
