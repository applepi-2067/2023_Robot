// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private final WPI_TalonFX m_leftMotor = new WPI_TalonFX(1);
  private final WPI_TalonFX m_rightMotor = new WPI_TalonFX(2);
  private final WPI_TalonFX m_leftMotorFollower = new WPI_TalonFX(3);
  private final WPI_TalonFX m_rightMotorFollower = new WPI_TalonFX(4);
  // private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  private final XboxController m_driverController = new XboxController(0);
  
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private static int nRotations = 0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() { 
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    /* factory default values */
    m_leftMotor.configFactoryDefault();
    m_rightMotor.configFactoryDefault();
    m_leftMotorFollower.configFactoryDefault();
    m_rightMotorFollower.configFactoryDefault();

    //enable motion magic
    configMotionMagic(m_leftMotor);
    configMotionMagic(m_rightMotor);

    //the .follow method tells the secondary motors to follow the commands of their respective motors
    m_leftMotorFollower.follow(m_leftMotor);
    m_rightMotorFollower.follow(m_rightMotor);

    /* flip values so robot moves forward when stick-forward/LEDs-green */
    m_rightMotor.setInverted(true);
    m_rightMotorFollower.setInverted(true);
  }

  public void configMotionMagic(WPI_TalonFX _talon) { 
    
    // /* Configure Sensor Source for Pirmary PID */
    _talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx,
        Constants.kTimeoutMs);

    /* set deadband to super small 0.001 (0.1 %).
      The default deadband is 0.04 (4 %) */
    _talon.configNeutralDeadband(0.001, Constants.kTimeoutMs);

    /**
     * Configure Talon FX Output and Sensor direction accordingly Invert Motor to
     * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
     * sensor to have positive increment when driving Talon Forward (Green LED)
     */
    _talon.setSensorPhase(false);
    _talon.setInverted(false);
    // /*
    //   * Talon FX does not need sensor phase set for its integrated sensor
    //   * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
    //   * and the user calls getSelectedSensor* to get the sensor's position/velocity.
    //   * 
    //   * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
    //   */
    //     // _talon.setSensorPhase(true);

    // /* Set relevant frame periods to be at least as fast as periodic rate */
    // _talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
    // _talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

    // /* Set the peak and nominal outputs */
    // _talon.configNominalOutputForward(0, Constants.kTimeoutMs);
    // _talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
    // _talon.configPeakOutputForward(1, Constants.kTimeoutMs);
    // _talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    // /* Set Motion Magic gains in slot0 - see documentation */
    // _talon.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
    // _talon.config_kF(Constants.kSlotIdx, Constants.kGains.kF, Constants.kTimeoutMs);
    // _talon.config_kP(Constants.kSlotIdx, Constants.kGains.kP, Constants.kTimeoutMs);
    // _talon.config_kI(Constants.kSlotIdx, Constants.kGains.kI, Constants.kTimeoutMs);
    // _talon.config_kD(Constants.kSlotIdx, Constants.kGains.kD, Constants.kTimeoutMs);

    // /* Set acceleration and vcruise velocity - see documentation */
    // _talon.configMotionCruiseVelocity(40000, Constants.kTimeoutMs);
    // _talon.configMotionAcceleration(4000, Constants.kTimeoutMs);

    /* Zero the sensor once on robot boot up */
    _talon.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //rotate wheel one revolution
    nRotations = 10;
    double targetTicks = nRotations * 2048 * 10.0;
    if (m_driverController.getAButton()) {
      double e = targetTicks - m_rightMotor.getSelectedSensorPosition();
      double p = 0;

      if (e > 1) {
        p = 1;
        
      } else if (e < 1) {
        p = -1;

      } else {
        p = 0;
      }

      System.out.println("e = " + e);

      m_rightMotor.set(TalonFXControlMode.PercentOutput, p);
    } else {
      m_rightMotor.set(TalonFXControlMode.PercentOutput, 0);
    }


  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
