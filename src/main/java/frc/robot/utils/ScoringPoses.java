package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Constants.ScoringInfo.ScoringOffsets;

public class ScoringPoses {
    private Pose2d m_robotPickupPiecePose2d;
    private Pose2d m_topCubeScoreRobotPose2d;
    private Pose2d m_topCubeScorePose2d;

    private double m_robotPickupPieceAbsoluteAngleDegrees;
    private double m_pickupWaistRotationDegrees;
    
    public ScoringPoses(boolean isBlue, boolean isTop) {
      int yCoeff = getYCoeff(isBlue, isTop);
      int aprilTagID = Constants.ScoringInfo.m_initialAprilTagID;

      m_robotPickupPiecePose2d = calcRobotPickupPiecePose2d(yCoeff, aprilTagID);
      m_topCubeScoreRobotPose2d = calcTopCubeScoreRobotPose2d(yCoeff, aprilTagID);
      m_topCubeScorePose2d = calcTopCubeScorePose2d(aprilTagID);

      m_robotPickupPieceAbsoluteAngleDegrees = calcRobotPickupPieceAbsoluteAngleDegrees(isBlue);
      m_pickupWaistRotationDegrees = calcPickupWaistRotationDegrees(isBlue, isTop);
    }
    
    private int getYCoeff(boolean isBlue, boolean isTop) {
      int yCoeff;
      if (isTop) {
        yCoeff = 1;
      }
      else {
        yCoeff = -1;
      }

      if (!isBlue) {
        yCoeff *= -1;
      }

      return yCoeff;
    }

    private Pose2d calcRobotPickupPiecePose2d(int yCoeff, int aprilTagID) {
      Pose2d tagOffsetPose2d = new Pose2d(
        ScoringOffsets.ROBOT_PICKUP_PIECE_X_OFFSET,
        ScoringOffsets.ROBOT_PICKUP_PIECE_Y_OFFSET * yCoeff,
        new Rotation2d()
      );

      Pose2d robotPickupPiecePose2d = Transforms.targetRelativePoseToAbsoluteFieldPose(aprilTagID, tagOffsetPose2d);
      return robotPickupPiecePose2d;
    }

    private Pose2d calcTopCubeScoreRobotPose2d(int yCoeff, int aprilTagID) {
      Pose2d tagOffsetPose2d = new Pose2d(
        ScoringOffsets.TOP_CUBE_SCORE_ROBOT_X_OFFSET,
        ScoringOffsets.TOP_CUBE_SCORE_ROBOT_Y_OFFSET * yCoeff,
        new Rotation2d()
      );

      Pose2d topCubeScoreRobotPose2d = Transforms.targetRelativePoseToAbsoluteFieldPose(aprilTagID, tagOffsetPose2d);
      return topCubeScoreRobotPose2d;
    }

    private Pose2d calcTopCubeScorePose2d(int aprilTagID) {
      Pose2d tagOffsetPose2d = new Pose2d(ScoringOffsets.TOP_CUBE_SCORE_X_OFFSET, 0.0, new Rotation2d());
      Pose2d topCubeScorePose2d = Transforms.targetRelativePoseToAbsoluteFieldPose(aprilTagID, tagOffsetPose2d);
      return topCubeScorePose2d;
    }

    private double calcRobotPickupPieceAbsoluteAngleDegrees(boolean isBlue) {
      if (isBlue) {
        return 180.0;
      }
      else {
        return 0.0;
      }
    }

    private double calcPickupWaistRotationDegrees(boolean isBlue, boolean isTop) {
      double pickupWaistRotationDegrees = 180.0;

      if (!isBlue) {
        pickupWaistRotationDegrees *= -1.0;
      }
      if (!isTop) {
        pickupWaistRotationDegrees *= -1.0;
      }

      return pickupWaistRotationDegrees;
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

    public double getRobotPickupPieceAbsoluteAngleDegrees() {
      return m_robotPickupPieceAbsoluteAngleDegrees;
    }

    public double getPickupWaistRotationDegrees() {
      return m_pickupWaistRotationDegrees;
    }
  }
