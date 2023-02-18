// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  public static Vision instance = null;
  private Drivetrain m_drivetrain = Drivetrain.getInstance();

  private PhotonCamera m_camera = new PhotonCamera("Arducam_1");
  private PhotonPoseEstimator m_photonPoseEstimator;

  private AprilTagFieldLayout aprilTagFieldLayout = null;
  
  // Get a new object through singleton method
  public static Vision getInstance() {
    if (instance == null) {
      instance = new Vision();
    }
    return instance;
  }

  // Constructor is private since this class is singleton
  private Vision() {
    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
      m_photonPoseEstimator = new PhotonPoseEstimator(
        aprilTagFieldLayout,
        PoseStrategy.LOWEST_AMBIGUITY,
        m_camera,
        new Transform3d()
      );
    }
    catch (IOException e) {
      System.out.println("Couldn't load April Tag Field Layout.");
    }
  }

  /**
   * Transform target relative pose (with target ID) to absolute field pose.
   * 
   * @param targetID: target ID.
   * @param targetRelativePose: pose in target coordinates.
   * @return: absolute field pose.
   */
  public Pose2d getAbsoluteFieldPoseFromTargetRelativePose(int targetID, Pose2d targetRelativePose) {
    Optional<Pose3d> result = aprilTagFieldLayout.getTagPose(targetID);
    Pose2d targetAbsolutePose = result.get().toPose2d();

    double targetAbsolutePoseX = targetAbsolutePose.getX();
    double targetAbsolutePoseY = targetAbsolutePose.getY();
    double targetAbsolutePoseRotation = targetAbsolutePose.getRotation().getRadians();    

    double targetRelativePoseX = targetRelativePose.getX();
    double targetRelativePoseY = targetRelativePose.getY();
    double targetRelativePoseRotation = targetRelativePose.getRotation().getRadians();
    
    double absoluteFieldPoseX = (
      targetAbsolutePoseX + 
      (targetRelativePoseX * Math.sin(targetRelativePoseRotation)) + 
      (targetRelativePoseY * Math.cos(targetRelativePoseRotation))
    );
    double absoluteFieldPoseY = (
      targetAbsolutePoseY + 
      (targetRelativePoseY * Math.sin(targetRelativePoseRotation)) + 
      (targetRelativePoseX * Math.cos(targetRelativePoseRotation))
    );
    double absoluteFieldPoseRotation = targetAbsolutePoseRotation + targetRelativePoseRotation;

    Pose2d absoluteFieldPose = new Pose2d(absoluteFieldPoseX, absoluteFieldPoseY, new Rotation2d(absoluteFieldPoseRotation));
    return absoluteFieldPose;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Optional<EstimatedRobotPose> result = m_photonPoseEstimator.update();
    if (result.isPresent()) {
      Pose2d estimatedRobotPose2d = result.get().estimatedPose.toPose2d();
      double timestampSeconds = result.get().timestampSeconds;
      m_drivetrain.addVisionMeaurement(estimatedRobotPose2d, timestampSeconds);
    }
  }
}
