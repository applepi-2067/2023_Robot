// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsDevices;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class ClawPneumatics
    extends SubsystemBase implements Loggable {

  private static ClawPneumatics instance = null;
  private Solenoid m_clawSolenoid = new Solenoid(PneumaticsDevices.MODULE_TYPE,
      PneumaticsDevices.CLAW);

  public static ClawPneumatics getInstance() {
    if (instance == null) {
      instance = new ClawPneumatics();
    }
    return instance;
  }

  /**
   * Creates a new ClawGrasp
   * .
   */
  private ClawPneumatics() {
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
    m_clawSolenoid.set(true);
  } 
  /* Close the Claw */
  public void close() { 
     m_clawSolenoid.set(false);
  }
}
