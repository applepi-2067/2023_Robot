// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

public class ClawBelt extends SubsystemBase implements Loggable {

  private static ClawBelt instance = null;


  public static ClawBelt getInstance() {
    if (instance == null) {
      instance = new ClawBelt();
    }
    return instance;
  }

  /** Creates a new ClawBelt. */
  private ClawBelt() {}

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
 
  @Config(rowIndex = 0, columnIndex = 5, width = 1, height = 1)
  public void clawIntake(boolean push) {
    
  }
}