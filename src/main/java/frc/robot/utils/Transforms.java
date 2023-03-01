package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class Transforms {

   /**
    * Transform target relative pose (with target ID) to absolute field pose.
    * 
    * @param targetID: target ID.
    * @param targetRelativePose: pose in target coordinates.
    * @return: absolute field pose.
    */
    public static Pose2d targetRelativePoseToAbsoluteFieldPose(int targetID, Pose2d targetRelativePose) {
        Optional<Pose3d> result = Constants.Field.aprilTagFieldLayout.getTagPose(targetID);
        Pose2d targetAbsolutePose = result.get().toPose2d();

        double targetAbsolutePoseX = targetAbsolutePose.getX();
        double targetAbsolutePoseY = targetAbsolutePose.getY();
        double targetAbsolutePoseRotation = targetAbsolutePose.getRotation().getRadians();    

        double targetRelativePoseX = targetRelativePose.getX();
        double targetRelativePoseY = targetRelativePose.getY();
        double targetRelativePoseRotation = targetRelativePose.getRotation().getRadians();
    
        double absoluteFieldPoseX = (
            targetAbsolutePoseX
            + (targetRelativePoseX * Math.cos(targetAbsolutePoseRotation))
            - (targetRelativePoseY * Math.sin(targetAbsolutePoseRotation))
        );
        double absoluteFieldPoseY = (
            targetAbsolutePoseY
            + (targetRelativePoseX * Math.sin(targetAbsolutePoseRotation))
            + (targetRelativePoseY * Math.cos(targetAbsolutePoseRotation))
        );
        double absoluteFieldPoseRotation = targetAbsolutePoseRotation + targetRelativePoseRotation;

        Pose2d absoluteFieldPose = new Pose2d(absoluteFieldPoseX, absoluteFieldPoseY, new Rotation2d(absoluteFieldPoseRotation));
        return absoluteFieldPose;
    }
}
