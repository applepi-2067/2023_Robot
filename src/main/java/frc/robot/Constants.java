// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.utils.Gains;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static class CANDeviceIDs {
    public static final int DT_MOTOR_LEFT_1_ID = 1;
    public static final int DT_MOTOR_RIGHT_1_ID = 2;
    public static final int DT_MOTOR_LEFT_2_ID = 3;
    public static final int DT_MOTOR_RIGHT_2_ID = 4;

    public static final int MOTOR_WAIST_ID = 5;

    public static final int PIGEON_TALON_ID = 11;
    public static final int ARM_MOTOR_ID = 8;
    public static final int MOTOR_SHOULDER_ID = 10;

    public static final int PIGEON_IMU_ID = 0;
  }

  public static class DiscreteInputs {
    public static final int ARM_END_OF_TRAVEL_DI = 0;

    public static final int PBOT_JUMPER_DI = 9;
  }

  public static class Drivetrain {
    /**
     * Which PID slot to pull gains from. Starting 2018, you can choose from
     * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
     * configuration.
     */
    public static final int MOTION_MAGIC_PID_SLOT = 0;
    public static final int TURNING_PID_SLOT = 1;

    /**
     * Talon FX supports multiple (cascaded) PID loops. For
     * now we just want the primary one.
     */
    public static final int kPIDLoopIdx = 0;

    /**
     * set to zero to skip waiting for confirmation, set to nonzero to wait and
     * report to DS if action fails.
     */
    public static final int kTimeoutMs = 30;

    /**
     * Gains used in Motion Magic, to be adjusted accordingly
     * Gains(kp, ki, kd, kf, izone, peak output);
     */
    public static final Gains kGains = new Gains(0.1, 0.001, 0.0, 0.0, 300, 1.0);
    public static final Gains turningGains = new Gains(2.0, 0.0, 4.0, 0.0, 200, 1.0);
  }

  public static final class PneumaticsDevices {
    public static final int CLAW_CLOSE = 12;
    public static final int CLAW_OPEN = 13;
    public static final PneumaticsModuleType MODULE_TYPE = PneumaticsModuleType.REVPH;
  }

}
