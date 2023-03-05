// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  public static Vision instance = null;
  private Drivetrain m_drivetrain = Drivetrain.getInstance();

  private PhotonCamera m_camera = new PhotonCamera("Arducam_1");
  private PhotonPoseEstimator m_photonPoseEstimator;
  
  // Thresholds for single target rejection
  private double MAX_TARGET_AMBIGUITY = 0.1;  // [0, 1]
  private double MAXIMUM_TARGET_DISTANCE_METERS = 2.0;

  public static Vision getInstance() {
    if (instance == null) {
      instance = new Vision();
    }
    return instance;
  }

  // Constructor is private since this class is singleton
  private Vision() {
    m_photonPoseEstimator = new PhotonPoseEstimator(
      Constants.Field.aprilTagFieldLayout,
      PoseStrategy.MULTI_TAG_PNP,
      m_camera,
      new Transform3d()
    );
    m_photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Optional<EstimatedRobotPose> result = m_photonPoseEstimator.update();
    if (result.isPresent()) {
      EstimatedRobotPose robotPose = result.get();
      Pose2d estimatedRobotPose2d = robotPose.estimatedPose.toPose2d();
      double timestampSeconds = result.get().timestampSeconds;
      
      // If we have more than one target in our estimate then we use MultiPNP to solve pose
      // which is extremely robust at any distance, so we juse use that measurement directly
      if (robotPose.targetsUsed.size() > 1) {
        m_drivetrain.addVisionMeaurement(estimatedRobotPose2d, timestampSeconds);
      } else {
        // Estimates based on a single tag must pass specific checks before being used
        PhotonTrackedTarget target = robotPose.targetsUsed.get(0);

        // Ignore single targets with ambiguity above threshold
        if (target.getPoseAmbiguity() > MAX_TARGET_AMBIGUITY) {
          return;
        }

        // Ignore single targets that are too far away
        Transform3d cameraToTarget3D = robotPose.targetsUsed.get(0).getBestCameraToTarget();
        double distanceToTarget = cameraToTarget3D.getTranslation().toTranslation2d().getNorm();
        if (distanceToTarget > MAXIMUM_TARGET_DISTANCE_METERS) {
          return;
        }

        // Target is good! Add it to pose estimate
        m_drivetrain.addVisionMeaurement(estimatedRobotPose2d, timestampSeconds);
      }
    }
  }
}
