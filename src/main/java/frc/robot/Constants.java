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

  public static class ZeroingOffsets {
    public static final double SHOULDER_FRONT_MINIMUM_ANGLE = -76.5;
    public static final double WAIST_ZERO_SENSOR_OFFSET = 14.0;  // Angle from waist zero sensor to true zero
  }

  public static class CANDeviceIDs {
    public static final int MOTOR_LEFT_1_ID = 1;
    public static final int MOTOR_RIGHT_1_ID = 2;
    public static final int MOTOR_LEFT_2_ID = 3;
    public static final int MOTOR_RIGHT_2_ID = 4;

    public static final int MOTOR_WAIST_ID = 5;
    public static final int ARM_MOTOR_ID = 8;
    public static final int MOTOR_SHOULDER_ID = 14;

    public static final int PIGEON_IMU_ID = 0;
  }

  public static class DiscreteInputs {
    public static final int WAIST_ZEROING_DI = 0;
    public static final int CLAW_IR_SENSOR_DI = 1;
    public static final int ARM_END_OF_TRAVEL_DI = 2;

    public static final int PBOT_JUMPER_DI = 9;
  }

  public static class Drivetrain {
    /**
     * Which PID slot to pull gains from. Starting 2018, you can choose from
     * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
     * configuration.
     */
    public static final int kPositionSlotIdx = 0;
    public static final int kVelocitySlotIdx = 1;

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
    public static final Gains kPositionGains = new Gains(0.1, 0.001, 0.0, 0.0, 300.0, 1.0);
    public static final Gains kVelocityGains = new Gains(0.1, 0.0, 0.0, 0.0, 0.0, 1.0); 

    // Maximum drivetrain velocity in meters per seconds.
    public static final double MAX_DRIVETRAIN_VELOCITY = 5.0;
    
    // Drivetrain only moves when abs(stick) > deadband. Compensates for stick drift.
    public static final double DRIVETRAIN_CONTROLLER_DEADBAND = 0.03;

    public static final double MOTOR_ACCELERATION = 5.0;
  }

  public static final class PneumaticsDevices {
    public static final int CLAW_CLOSE = 0;
    public static final int CLAW_OPEN = 1;
    public static final PneumaticsModuleType MODULE_TYPE = PneumaticsModuleType.CTREPCM;
  }

  public static final class SetpointTolerances {
    public static final double SHOULDER_ANGLE_TOLERANCE = 0.1;
    public static final double ARM_METERS_TOLERANCE = 0.005;
    public static final double WAIST_ANGLE_TOLERANCE = 0.1;
  }
  
  public static final class IKOffsets {
    public static final double MINIMUM_ARM_LENGTH = 0.5334;  // Arm length when at zero
    public static final double SHOULDER_HEIGHT = 0.9779;
  }
  public static final class IKConstraints {
    public static final double MINIMUM_Z_HEIGHT = 0.28; 
  }
}
