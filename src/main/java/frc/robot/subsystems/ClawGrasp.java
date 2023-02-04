// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class ClawGrasp
 extends SubsystemBase implements Loggable {






  private static ClawGrasp
   instance = null;


  public static ClawGrasp
   getInstance() {
    if (instance == null) {
      instance = new ClawGrasp
      ();
    }
    return instance;
  }

  /** Creates a new ClawGrasp
   * . */
  private ClawGrasp
  () {}

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  public void open() {
    
  }
}