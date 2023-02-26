// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;

public class Lights extends SubsystemBase {

  private static Lights instance = null;
  public Relay m_rgbLight = new Relay(0);
  public static Lights getInstance() {
    if (instance == null) {
      instance = new Lights();
    }
    return instance;
  }


  public void purple() {
  m_rgbLight.set(Value.kForward);
  }

  public void yellow() {
    m_rgbLight.set(Value.kReverse);
  }

  public void white() {
   m_rgbLight.set(Value.kOff);
  }

  public void off() {
    m_rgbLight.set(Value.kOn);
  }

  @Override
  public void periodic() {

  }
}
