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
import edu.wpi.first.math.geometry.Pose3d;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  private PhotonCamera m_camera = new PhotonCamera("Arducam_1");

  /** Creates a new Vision. */
  public Vision() {}

  /**
   * Provided Pose2d relative to target, returns Pose2d relative to camera
   * 
   * @param targetID: photonvision target ID
   * @param destination: Pose2d relative to target
   * @return: Pose2d relative to camera
   */
  public Pose2d getCameraToDestTranslation(int targetID, Pose2d destination) {
    Transform3d cameraToTargetTransform = getCameraToTargetTransform(targetID);
    if (cameraToTargetTransform != null) {
      Transform3d targetToCameraTransform = cameraToTargetTransform.inverse();
      Pose3d destination3dTargetSpace = new Pose3d(destination);
      Pose3d destination3dCameraSpace = destination3dTargetSpace.transformBy(targetToCameraTransform);
      Pose2d destination2dCameraSpace = destination3dCameraSpace.toPose2d();
      return destination2dCameraSpace;
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
  public Transform3d getCameraToTargetTransform(int targetID) {
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
  public PhotonTrackedTarget getTarget(List<PhotonTrackedTarget> targets, int targetID) {
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
