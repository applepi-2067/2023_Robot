package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class ScoringPoses {
    private Pose2d m_robotPickupPiecePose2d;
    private Pose2d m_topCubeScoreRobotPose2d;
    private Pose2d m_topCubeScorePose2d;
    
    public ScoringPoses(boolean isBlue, boolean isTop) {
      int yCoeff = getYCoeff(isBlue, isTop);
      int aprilTagID = getAprilTagID(isBlue, isTop);

      m_robotPickupPiecePose2d = calcRobotPickupPiecePose2d(yCoeff, aprilTagID);
      m_topCubeScoreRobotPose2d = calcTopCubeScoreRobotPose2d(yCoeff, aprilTagID);
      m_topCubeScorePose2d = calcTopCubeScorePose2d(aprilTagID);
    }
    
    private int getYCoeff(boolean isBlue, boolean isTop) {
      int yCoeff;
      if (isTop) {
        yCoeff = 1;
      }
      else {
        yCoeff = -1;
      }

      return yCoeff;
    }

    private int getAprilTagID(boolean isBlue, boolean isTop) {
      int aprilTagID ;
      if (isBlue) {
        if (isTop) {
          aprilTagID = 6;
        }
        else {
          aprilTagID = 8;
        }
      }

      else {
        if (isTop) {
          aprilTagID = 3;
        }
        else {
          aprilTagID = 1;
        }
      }

      return aprilTagID;
    }

    private Pose2d calcRobotPickupPiecePose2d(int yCoeff, int aprilTagID) {
      Pose2d robotPickupPiecePose2d = Transforms.targetRelativePoseToAbsoluteFieldPose(
        aprilTagID, new Pose2d(Units.inchesToMeters(100.0), Units.inchesToMeters(10.0 * yCoeff), new Rotation2d()) // TODO: Fix offsets
      );
      return robotPickupPiecePose2d;
    }

    private Pose2d calcTopCubeScoreRobotPose2d(int yCoeff, int aprilTagID) {
      Pose2d topCubeScoreRobotPose2d = Transforms.targetRelativePoseToAbsoluteFieldPose(
        aprilTagID, new Pose2d(Units.inchesToMeters(10.0), Units.inchesToMeters(10.0 * yCoeff), new Rotation2d()) // TODO: Fix offsets
      );
      return topCubeScoreRobotPose2d;
    }

    private Pose2d calcTopCubeScorePose2d(int aprilTagID) {
      Pose2d topCubeScorePose2d = Transforms.targetRelativePoseToAbsoluteFieldPose(
        aprilTagID, new Pose2d(Units.inchesToMeters(-27.0), 0.0, new Rotation2d())
      );
      return topCubeScorePose2d;
    }

    public Pose2d getRobotPickupPiecePose2d() {
      return m_robotPickupPiecePose2d;
    }

    public Pose2d getTopCubeScoreRobotPose2d() {
      return m_topCubeScoreRobotPose2d;
    }

    public Pose2d getTopCubeScorePose2d() {
      return m_topCubeScorePose2d;
    }
  }
