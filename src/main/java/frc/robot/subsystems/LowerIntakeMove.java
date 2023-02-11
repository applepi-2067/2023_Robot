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

public class LowerIntakeMove
    extends SubsystemBase implements Loggable {

  private static LowerIntakeMove instance = null;
  private DoubleSolenoid m_intakSolenoid = new DoubleSolenoid(PneumaticsDevices.MODULE_TYPE,
      PneumaticsDevices.INTAKE_IN,
      PneumaticsDevices.INTAKE_OUT);

  public static LowerIntakeMove getInstance() {
    if (instance == null) {
      instance = new LowerIntakeMove();
    }
    return instance;
  }

  /**
   * Creates a new ClawGrasp
   * .
   */
  private LowerIntakeMove() {
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * Open the claw
   */
  public void open() {
  }
  /* Close the Claw */
  public void close() { 
  }
}
