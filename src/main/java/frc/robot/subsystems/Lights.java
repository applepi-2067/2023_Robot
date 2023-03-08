// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Relay.Value;
import frc.robot.MonkeyTest;

public class Lights extends SubsystemBase {

  public enum Color {
    WHITE,
    YELLOW,
    PURPLE
  }

  private static Lights instance = null;
  public Relay m_rgbLight = new Relay(0);
  private Color m_color;
  private boolean m_on;  // Light strip on or off
  private boolean m_blink;  // Blink
  private double m_blinkPeriod;  // Blink interval in seconds
  private Timer m_timer = new Timer();

  private Lights() {
    m_color = Color.WHITE;
    m_on = false;
    m_blink = false;
    m_blinkPeriod = 1.0;
  }

  public static Lights getInstance() {
    if (instance == null) {
      instance = new Lights();
    }
    return instance;
  }

  public void setColor(Color color) {
    m_color = color;
    setRelayColor();
  }

  public void on() {
    if (MonkeyTest.get() == 5) {
      m_on = false;
      m_rgbLight.set(Value.kOn);
      return;
    }
    m_on = true;
    setRelayColor();
  }

  public void off() {
    m_on = false;
    m_rgbLight.set(Value.kOn);
  }

  private void setRelayColor() {
    if (!m_on) {
      return;
    }
    switch (m_color) {
      case PURPLE:
        m_rgbLight.set(Value.kForward);
        break;

      case YELLOW:
        m_rgbLight.set(Value.kReverse);
        break;

      case WHITE:
        m_rgbLight.set(Value.kOff);
        break;
    }
  }

  /**
   * Set the blink period in seconds
   * @param period Blink period in seconds
   */
  public void blink(double period) {
    m_blink = true;
    m_blinkPeriod = period;
    m_timer.restart();
  }

  public void disableBlink() {
    m_blink = false;
    m_timer.stop();
  }

  @Override
  public void periodic() {
    if (!m_blink) {
      return;
    }

    if (m_timer.hasElapsed(m_blinkPeriod)) {
      if (m_on) {
        off();
      } else {
        on();
      }
      m_timer.restart();
    }
  }
}
