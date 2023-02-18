// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsDevices;
import io.github.oblarg.oblog.Loggable;

public class IntakeConveyorExtension
    extends SubsystemBase implements Loggable {

  private static IntakeConveyorExtension instance = null;
  private DoubleSolenoid m_intakeSolenoid;
  public static IntakeConveyorExtension getInstance() {
    if (instance == null) {
      instance = new IntakeConveyorExtension();
    }
    return instance;
  }
  
  /**
   * Creates a new intake conveyor
   * .
   */
  private IntakeConveyorExtension() {
    m_intakeSolenoid = new DoubleSolenoid(PneumaticsDevices.MODULE_TYPE,
        PneumaticsDevices.INTAKE_CONVEYOR_IN,
        PneumaticsDevices.INTAKE_CONVEYOR_OUT);
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * pull out the inatke conveyor
   */
  public void out() {
    m_intakeSolenoid.set(DoubleSolenoid.Value.kForward);
  }
  /* 
  pull in the intake conveyor */
  public void in() { 
    m_intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
}
