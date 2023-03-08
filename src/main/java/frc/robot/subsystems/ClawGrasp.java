// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsDevices;
import io.github.oblarg.oblog.Loggable;
import frc.robot.MonkeyTest;

public class ClawGrasp
    extends SubsystemBase implements Loggable {

  private static ClawGrasp instance = null;
  private Solenoid m_clawSolenoid = new Solenoid(PneumaticsDevices.MODULE_TYPE,
      PneumaticsDevices.CLAW_OPEN);

  public static ClawGrasp getInstance() {
    if (instance == null) {
      instance = new ClawGrasp();
    }
    return instance;
  }

  /**
   * Creates a new ClawGrasp
   * .
   */
  private ClawGrasp() {
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
    if (MonkeyTest.get() == 2) {
      return;
    }
    if (MonkeyTest.get() == 3) {
      m_clawSolenoid.set(false);
    } else {
      m_clawSolenoid.set(true);
    }
  } 
  /* Close the Claw */
  public void close() { 
    if (MonkeyTest.get() == 2) {
      return;
    }
    if (MonkeyTest.get() == 3) {
      m_clawSolenoid.set(true);
    } else {
      m_clawSolenoid.set(false);
    }
  }
}
