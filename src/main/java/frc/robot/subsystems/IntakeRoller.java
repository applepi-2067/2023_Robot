// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

public class IntakeRoller extends SubsystemBase implements Loggable {
  
  private static IntakeRoller instance = null;


  public static IntakeRoller getInstance() {
    if (instance == null) {
      instance = new IntakeRoller();
    }
    return instance;
  }

  /** Creates a new Intake. */
  private IntakeRoller() {}

  @Override
  public void periodic() {

  }

}