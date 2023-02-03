// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  private PhotonCamera m_camera = new PhotonCamera("Arducam_1");
  public static Vision instance = null;
  private Vision() {}  // Constructor is private since this class is singleton

  public static Vision getInstance() {  // Get a new object through singleton method
    if (instance == null) {
      instance = new Vision();
    }
    return instance;
  }

  /**
   * Provided Pose2d relative to target, returns Pose2d relative to camera
   * 
   * @param targetID: photonvision target ID
   * @param destination: Pose2d relative to target
   * @return: Pose2d relative to camera
   */
  public Pose2d getCameraToDestPose(int targetID, Pose2d destination) {
    Transform3d cameraToTargetTransform = getCameraToTargetTransform(targetID);
    if (cameraToTargetTransform != null) {
      double targetX = cameraToTargetTransform.getX();
      double targetY = cameraToTargetTransform.getY();
      double targetRotationRadians = cameraToTargetTransform.getRotation().toRotation2d().getRadians();

      // Coords of destination in target reference frame.
      double destinationXTarget = destination.getX();
      double destinationYTarget = destination.getY();
      double destinationRotationRadiansTarget = destination.getRotation().getRadians();

      // Coords of destination in camera reference frame.
      double destinationXCamera = targetX + (destinationXTarget * Math.cos(targetRotationRadians)) + (destinationYTarget * Math.sin(targetRotationRadians));
      double destinationYCamera = targetY + (destinationYTarget * Math.cos(targetRotationRadians)) + (destinationXTarget * Math.sin(targetRotationRadians));
      double destinationRotationRadiansCamera = targetRotationRadians + destinationRotationRadiansTarget;

      Pose2d cameraToDestPose = new Pose2d(destinationXCamera, destinationYCamera, new Rotation2d(destinationRotationRadiansCamera));
      return cameraToDestPose;
    }

    return null;
  }

  /**
   * Get camera to target transform.
   * Return null if the target isn't tracked.
   * 
   * @param targetID: photonvision target ID
   * @return camera to target Transform3d or null
   */
  private Transform3d getCameraToTargetTransform(int targetID) {
    PhotonPipelineResult result = m_camera.getLatestResult();
    if (result.hasTargets()) {
      List<PhotonTrackedTarget> targets = result.getTargets();
      PhotonTrackedTarget target = getTarget(targets, targetID);

      if (target != null) {
        return target.getBestCameraToTarget();
      }
    }

    return null;
  }

  /**
   * Return the target with the specified ID, or null
   * 
   * @param targets: list of tracked targets
   * @param targetID: photonvision target ID
   * @return target or null if the camera doesn't see the correct target
   */
  private PhotonTrackedTarget getTarget(List<PhotonTrackedTarget> targets, int targetID) {
    for (PhotonTrackedTarget target: targets) {
      if (target.getFiducialId() == targetID) {
        return target;
      }
    }

    return null;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
