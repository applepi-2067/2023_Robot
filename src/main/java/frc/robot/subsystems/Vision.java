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
  private Pose3d m_lastCameraPoseAbsolute = new Pose3d();
  
  // Get a new object through singleton method
  public static Vision getInstance() {
    if (instance == null) {
      instance = new Vision();
    }
    return instance;
  }

  // Constructor is private since this class is singleton
  private Vision() {
    AprilTagFieldLayout aprilTagFieldLayout = null;
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

  // TODO: Transform destination pose relative to target (give ID) to absolute field pose.
  // public Pose2d getCameraToDestPose(int targetID, Pose2d destination) {
  //   double targetX = cameraToTargetTransform.getX();
  //   double targetY = cameraToTargetTransform.getY();
  //   double targetRotationRadians = cameraToTargetTransform.getRotation().toRotation2d().getRadians();

  //   // Coords of destination in target reference frame.
  //   double destinationXTarget = destination.getX();
  //   double destinationYTarget = destination.getY();
  //   double destinationRotationRadiansTarget = destination.getRotation().getRadians();

  //   // Coords of destination in camera reference frame.
  //   double destinationXCamera = targetX + (destinationXTarget * Math.cos(targetRotationRadians)) + (destinationYTarget * Math.sin(targetRotationRadians));
  //   double destinationYCamera = targetY + (destinationYTarget * Math.cos(targetRotationRadians)) + (destinationXTarget * Math.sin(targetRotationRadians));
  //   double destinationRotationRadiansCamera = targetRotationRadians + destinationRotationRadiansTarget;

  //   Pose2d cameraToDestPose = new Pose2d(destinationXCamera, destinationYCamera, new Rotation2d(destinationRotationRadiansCamera));
  //   return cameraToDestPose;
  // }

  public Pose3d getCameraAbsolutePose() {
    return m_lastCameraPoseAbsolute;
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
