// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  private PhotonCamera m_camera = new PhotonCamera("Arducam_1");

  /** Creates a new Vision. */
  public Vision() {}

    /**
   * Return relative position of target to camera.
   * 
   * @param targetID: photonvision target ID
   * @return relative (x, y) of target from camera
   */
  public double[] getCameraToTarget(int targetID) {
    PhotonPipelineResult result = m_camera.getLatestResult();
    if (result.hasTargets()) {
      List<PhotonTrackedTarget> targets = result.getTargets();
      PhotonTrackedTarget target = getTarget(targets, targetID);

      if (target != null) {
        Transform3d cameraToTargetTransform = target.getBestCameraToTarget();
        double targetX = cameraToTargetTransform.getX();
        double targetY = cameraToTargetTransform.getY();
        
        double[] targetCoords = {targetX, targetY};
        return targetCoords;
      }
    }

    return new double[0];
  }

  /**
   * Return relative position of destination to camera.
   * 
   * @param targetID: photonvision target ID
   * @param targetToDestX: photonvision target vertical displacement to destination (+ up)
   * @param targetToDestY: photonvision target vertical displacement to destination (+ left)
   * @return relative (x, y) of destination from camera
   */
  public double[] getCameraToDest(int targetID, double targetToDestX, double targetToDestY) {
    double[] targetCoords = getCameraToTarget(targetID);
    if (targetCoords.length == 2) {
      double destX = targetCoords[0] + targetToDestX;
      double destY = targetCoords[1] + targetToDestY;
        
      double[] destCoords = {destX, destY};
      return destCoords;
    }

    return new double[0];
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
