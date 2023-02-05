// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

public class Intake extends SubsystemBase implements Loggable {
  
  private static Intake instance = null;


  public static Intake getInstance() {
    if (instance == null) {
      instance = new Intake();
    }
    return instance;
  }

  /** Creates a new Intake. */
  private Intake() {}

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
 
  @Config(rowIndex = 0, columnIndex = 5, width = 1, height = 1)
  public void intake(boolean push) {
    
  }
}