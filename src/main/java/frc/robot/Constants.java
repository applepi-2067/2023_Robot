// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.commands.IK.IKCoordinate;
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
    public static final double SHOULDER_FRONT_MINIMUM_ANGLE = -62.5;  //  Angle at which the sensor stops detecting the magnet
    public static final double WAIST_ZERO_SENSOR_OFFSET = 11.64;  // Angle from waist zero sensor to true zero
  }

  public static class Poses {
    public static final double SHOULDER_STOW_ANGLE = -60.0;  // degrees
  }

  public static class CANDeviceIDs {
    public static final int DT_MOTOR_LEFT_1_ID = 1;
    public static final int DT_MOTOR_RIGHT_1_ID = 2;
    public static final int DT_MOTOR_LEFT_2_ID = 3;
    public static final int DT_MOTOR_RIGHT_2_ID = 4;
    public static final int WAIST_MOTOR_ID = 5;
    public static final int INTAKE_RIGHT_ROLLER_MOTOR_ID = 6;
    public static final int INTAKE_LEFT_ROLLER_MOTOR_ID = 7;
    public static final int ARM_MOTOR_ID = 8;
    public static final int INTAKE_CONVEYOR_MOTOR_ID = 9;
    public static final int CLAWBELT_MOTOR_ID = 10;

    public static final int INTAKE_LEFT_EXTENSION_MOTOR_ID = 12;
    public static final int INTAKE_RIGHT_EXTENSION_MOTOR_ID = 13;
    public static final int SHOULDER_MOTOR_ID = 14;

    public static final int INTAKE_EXTENSION_MOTOR_LEFT_ID = 16;
    public static final int INTAKE_EXTENSION_MOTOR_RIGHT_ID = 17;

    public static final int PIGEON_IMU_ID = 0;
  }

  public static class DiscreteInputs {
    public static final int WAIST_ZEROING_DI = 0;
    public static final int CLAW_IR_SENSOR_DI = 1;
    public static final int ARM_END_OF_TRAVEL_DI = 2;
    public static final int SHOULDER_ZEROING_DI = 8;
  }

  public static class Drivetrain {
    public static final Pose2d INITIAL_ROBOT_POSE2D = new Pose2d(12.813, 6.745, new Rotation2d(Math.PI)); // 3 meters from id 4, facing away.

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
    public static final Gains kPositionGains = new Gains(0.035, 0.0, 0.0, 0.0, 0.0, 1.0);
    // public static final Gains kPositionGains = new Gains(0.013, 0.0001, 0.0, 0.0, 200.0, 1.0);
    public static final Gains kVelocityGains = new Gains(0.1, 0.0, 0.0, 0.0, 0.0, 1.0); 

    // Maximum drivetrain velocity in meters per seconds.
    public static final double MAX_DRIVETRAIN_VELOCITY = 5.0;
    
    // Drivetrain only moves when abs(stick) > deadband. Compensates for stick drift.
    public static final double DRIVETRAIN_CONTROLLER_DEADBAND = 0.03;

    public static final double MOTOR_ACCELERATION = 5.0;  // m/s^2
    public static final double MOTOR_TURN_ACCELERATION = 7.0;  // m/s^2, speed differential of the wheels
    public static final double MOTOR_ACCELERATION_AUTO = 3.0;  // m/s^2
  }

  public static final class PneumaticsDevices {
    public static final int INTAKE_CONVEYOR_IN = 0;
    public static final int INTAKE_CONVEYOR_OUT = 1;
    public static final int CLAW_OPEN = 7;

    public static final PneumaticsModuleType MODULE_TYPE = PneumaticsModuleType.CTREPCM;
  }

  public static final class SetpointTolerances {
    public static final double SHOULDER_ANGLE_TOLERANCE = 1;
    public static final double ARM_METERS_TOLERANCE = 0.005;
    public static final double WAIST_ANGLE_TOLERANCE = 0.1;

    // Tolerance for auto position drive commands to end 
    public static final double AUTO_DISTANCE_TOLERANCE = 0.08;  // meters
    public static final double AUTO_VELOCITY_TOLERANCE = 0.2;  // m/s
  }
  
  public static final class IKPositions {
    public static final IKCoordinate ABOVE_INTAKE_BEFORE_ACQUISITION = new IKCoordinate(0.26, 0.0, 0.38); //TODO: SET
    public static final IKCoordinate ACQUIRING_PIECE_FROM_INTAKE = new IKCoordinate(0.23, 0.0, 0.27); //TODO: SET
    public static final IKCoordinate STOWED_WITH_GAME_PIECE_CLEAR_OF_INTAKE = new IKCoordinate(0.23, 0, 0.5); //TODO: SET

    public static final IKCoordinate HIGH_SCORING_POSITION = new IKCoordinate(-1.44, 0, 1.36); //over the back
    public static final IKCoordinate MID_SCORING_POSITION = new IKCoordinate(-1.00, 0, 1.03); //over the back
    public static final IKCoordinate LOW_SCORING_POSITION = new IKCoordinate(0.6858, 0, 0.2158); //TODO: SET
  }

  public static final class IKOffsets {
    public static final double MINIMUM_ARM_LENGTH = 0.5334;  // Arm length when at zero
    public static final double SHOULDER_HEIGHT = 0.9779;
  }

  public static final class IKConstraints {
    public static final double MINIMUM_Z_HEIGHT = 0.28;
  }

  public static final class Camera {
    public static final double CAMERA_HYPOTENUSE_OFFSET = Units.inchesToMeters(6.5);
  }

  public static final class Field {
    public static AprilTagFieldLayout aprilTagFieldLayout = loadFieldLayout();

    public static AprilTagFieldLayout loadFieldLayout() {
      try {
        return AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
      }
      catch (IOException e) {
        System.out.println("Couldn't load April Tag Field Layout.");
        return null;
      }
    }
  }
}
