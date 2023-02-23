package frc.robot.utils;

import edu.wpi.first.math.util.Units;

public class InverseKinematics {
    public static double getArmLength(double x, double y, double z) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2));
    }

    public static double getThetaShoulder(double x, double y, double z) {
        return Units.radiansToDegrees(Math.atan2(z, Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2))));
    }

    public static double getThetaWaist(double x, double y, double z) {
        return Units.radiansToDegrees(Math.atan2(y, x));
    }
    
}
