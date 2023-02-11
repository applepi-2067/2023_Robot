// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsDevices;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class IntakeConveyor
    extends SubsystemBase implements Loggable {

  private static IntakeConveyor instance = null;
  private DoubleSolenoid m_intakSolenoid = new DoubleSolenoid(PneumaticsDevices.MODULE_TYPE,
      PneumaticsDevices.INTAKE_CONVEYOR_IN,
      PneumaticsDevices.INTAKE_CONVEYOR_OUT);

  public static IntakeConveyor getInstance() {
    if (instance == null) {
      instance = new IntakeConveyor();
    }
    return instance;
  }

  /**
   * Creates a new intake conveyor
   * .
   */
  private IntakeConveyor() {
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
    m_intakSolenoid.set(DoubleSolenoid.Value.kForward);
  }
  /* pull in the intake conveyor */
  public void in() { 
    m_intakSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
}
