package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class PresetPoses {
    
  public enum AutoChoice {
    TOP,
    CENTER,
    BOTTOM,
    SCORE_PRELOADED_PIECE,
    ZERO_ALL,
    NONE
  }  

  public static final class InitialPoses {

    public static final double BLUE_START_X = 1.806;
    public static final double RED_START_X = 14.722;

    public static final double TOP_START_Y = 4.962;
    public static final double CENTER_START_Y = 2.206;
    public static final double BOTTOM_START_Y = 0.539;


    public static final class Blue {
      public static final Pose2d TOP_POSE2D = new Pose2d(BLUE_START_X, TOP_START_Y, new Rotation2d());
      public static final Pose2d CENTER_POSE2D = new Pose2d(BLUE_START_X, CENTER_START_Y, new Rotation2d());
      public static final Pose2d BOTTOM_POSE2D = new Pose2d(BLUE_START_X, BOTTOM_START_Y, new Rotation2d());
    }

    public static final class Red {
      public static final Pose2d TOP_POSE2D = new Pose2d(RED_START_X, TOP_START_Y, new Rotation2d());
      public static final Pose2d CENTER_POSE2D = new Pose2d(RED_START_X, CENTER_START_Y, new Rotation2d());
      public static final Pose2d BOTTOM_POSE2D = new Pose2d(RED_START_X, BOTTOM_START_Y, new Rotation2d());
    }
  }

  public static final class AutoPoses {

    public static final double BLUE_CUBE_PICKUP_X = 6.528;
    public static final double RED_CUBE_PICKUP_X = 10.025;

    public static final double TOP_CUBE_PICKUP_Y = 4.561;
    public static final double TOP_CUBE_SCORE_Y = 4.411;

    public static final double CENTER_CUBE_PICKUP_Y = 2.155;

    public static final double BOTTOM_CUBE_PICKUP_Y = 0.940;
    public static final double BOTTOM_CUBE_SCORE_Y = 1.090;

    public static final class Blue {
      public static final class Top {
        public static final Pose2d CUBE_PICKUP_POSE2D = new Pose2d(BLUE_CUBE_PICKUP_X, TOP_CUBE_PICKUP_Y, new Rotation2d());
        public static final Pose2d CUBE_SCORE_POSE2D = new Pose2d(InitialPoses.BLUE_START_X, TOP_CUBE_SCORE_Y, new Rotation2d());

        public static final double CUBE_PICKUP_WAIST_ROTATION_DEGREES = 180.0;
        public static final double CUBE_PICKUP_ROBOT_ABSOLUTE_ANGLE_DEGREES = 180.0;
      }
      public static final class Center {
        public static final Pose2d CUBE_PICKUP_POSE2D = new Pose2d(BLUE_CUBE_PICKUP_X, CENTER_CUBE_PICKUP_Y, new Rotation2d());
      }
      public static final class Bottom {
        public static final Pose2d CUBE_PICKUP_POSE2D = new Pose2d(BLUE_CUBE_PICKUP_X, BOTTOM_CUBE_PICKUP_Y, new Rotation2d());
        public static final Pose2d CUBE_SCORE_POSE2D = new Pose2d(InitialPoses.BLUE_START_X, BOTTOM_CUBE_SCORE_Y, new Rotation2d());

        public static final double CUBE_PICKUP_WAIST_ROTATION_DEGREES = -180.0;
        public static final double CUBE_PICKUP_ROBOT_ABSOLUTE_ANGLE_DEGREES = 180.0;
      }
    }

    public static final class Red {
      public static final class Top {
        public static final Pose2d CUBE_PICKUP_POSE2D = new Pose2d(RED_CUBE_PICKUP_X, TOP_CUBE_PICKUP_Y, new Rotation2d());
        public static final Pose2d CUBE_SCORE_POSE2D = new Pose2d(InitialPoses.RED_START_X, TOP_CUBE_SCORE_Y, new Rotation2d());

        public static final double CUBE_PICKUP_WAIST_ROTATION_DEGREES = -180.0;
        public static final double CUBE_PICKUP_ROBOT_ABSOLUTE_ANGLE_DEGREES = 0.0;
      }
      public static final class Center {
        public static final Pose2d CUBE_PICKUP_POSE2D = new Pose2d(RED_CUBE_PICKUP_X, CENTER_CUBE_PICKUP_Y, new Rotation2d());
      }
      public static final class Bottom {
        public static final Pose2d CUBE_PICKUP_POSE2D = new Pose2d(RED_CUBE_PICKUP_X, BOTTOM_CUBE_PICKUP_Y, new Rotation2d());
        public static final Pose2d CUBE_SCORE_POSE2D = new Pose2d(InitialPoses.RED_START_X, BOTTOM_CUBE_SCORE_Y, new Rotation2d());

        public static final double CUBE_PICKUP_WAIST_ROTATION_DEGREES = 180.0;
        public static final double CUBE_PICKUP_ROBOT_ABSOLUTE_ANGLE_DEGREES = 0.0;
      }
    }
  }
}
