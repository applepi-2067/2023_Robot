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
        return shiftAbsolutePoseByRelativePose(targetAbsolutePose, targetRelativePose);
    }

    /**
     * Shift a pose in absolute field space by a shift defined in the absolutePose's frame of reference
     * @param absolutePose Pose in absolute field space
     * @param relativeShift Shift defined from the perspective of the absolutePose
     * @return Shifted pose in field absolute space
     */
    public static Pose2d shiftAbsolutePoseByRelativePose(Pose2d absolutePose, Pose2d relativeShift) {
        double absolutePoseX = absolutePose.getX();
        double absolutePoseY = absolutePose.getY();
        double absolutePoseRotation = absolutePose.getRotation().getRadians();    

        double relativeShiftX = relativeShift.getX();
        double relativeShiftY = relativeShift.getY();
        double relativeShiftRotation = relativeShift.getRotation().getRadians();
    
        double absoluteFieldPoseX = (
            absolutePoseX
            + (relativeShiftX * Math.cos(absolutePoseRotation))
            - (relativeShiftY * Math.sin(absolutePoseRotation))
        );
        double absoluteFieldPoseY = (
            absolutePoseY
            + (relativeShiftX * Math.sin(absolutePoseRotation))
            + (relativeShiftY * Math.cos(absolutePoseRotation))
        );
        double absoluteFieldPoseRotation = absolutePoseRotation + relativeShiftRotation;

        return new Pose2d(absoluteFieldPoseX, absoluteFieldPoseY, new Rotation2d(absoluteFieldPoseRotation));
    }
}
