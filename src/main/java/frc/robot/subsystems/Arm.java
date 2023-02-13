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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Gains;
import frc.robot.utils.Util;
import frc.robot.Constants;
import frc.robot.Constants.CANDeviceIDs;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Arm extends SubsystemBase implements Loggable{

  private static Arm instance = null;

  private final CANSparkMax m_motor;
  private final SparkMaxPIDController m_pidController;
  private final RelativeEncoder m_encoder;
  private final DigitalInput m_endOfTravelSensor;

  private static final int CURRENT_LIMIT = 10; //Amps
  private static final double GEAR_RATIO = 5.0 * 4.0 * (18.0/34.0);
  private static final double OUTPUT_SPROCKET_PITCH_DIAMETER_METERS = 0.020574;
  private static final double RIGGING_EXTENSION_RATIO = 2.0;
  private static final double METERS_PER_REV = Math.PI * OUTPUT_SPROCKET_PITCH_DIAMETER_METERS * RIGGING_EXTENSION_RATIO;
  private static final boolean INVERT_MOTOR = false;

  private static double max_voltage_open_loop = 1.0;

  // PID Coefficients.
  private Gains gains = new Gains(0.1, 1e-4, 1, 0, 1, 1.0);

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
    m_pidController.setReference(metersToMotorRotations(meters), CANSparkMax.ControlType.kPosition);
  }

  /**
   * Set motor speed.
   * @param speed (-1.0 to 1.0)
   */
  public void setSpeed(double speed) {
    Util.limit(speed);
    m_pidController.setReference(speed * max_voltage_open_loop, CANSparkMax.ControlType.kVoltage);
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

  @Config (name = "Output Limit (V)", rowIndex = 2, columnIndex = 2, defaultValueNumeric = 1.0)
  public void setMaxVoltage(double v) {
    max_voltage_open_loop = v;
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
