// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeExtension extends SubsystemBase {

  private static IntakeExtension instance = null;
  

  public static IntakeExtension getInstance() {
     if(instance == null) {
      instance = new IntakeExtension();
     }
      return instance;
  }

    private IntakeExtension(){}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
