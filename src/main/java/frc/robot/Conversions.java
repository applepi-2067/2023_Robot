package frc.robot;

public class Conversions {

    public static double metersToMotorRotations(double meters, double METERS_PER_REV, double GEAR_RATIO) {
        return meters/METERS_PER_REV * GEAR_RATIO;
    }
    
    public static double motorRotationsToMeters(double rotations, double GEAR_RATIO, double METERS_PER_REV) {
        return (rotations / GEAR_RATIO) * METERS_PER_REV;
    } 

    public static double ticksPer100ms(double TICKS_PER_REV, double GEAR_RATIO) {
        return (TICKS_PER_REV / GEAR_RATIO);
    }

    public static double pigeonRotationsToDegrees(double PIGEON_UNITS_PER_ROTATION) {
        return (PIGEON_UNITS_PER_ROTATION / 360);
    }

    public static double metersToTicks(double setpoint, double TICKS_PER_REV, double GEAR_RATIO, double WHEEL_CIRCUMFERENCE_METERS) {
        return (setpoint * TICKS_PER_REV * GEAR_RATIO) / WHEEL_CIRCUMFERENCE_METERS;
    }
    
    public static double ticksToMeters(double setpoint, double WHEEL_CIRCUMFERENCE_METERS, double TICKS_PER_REV, double GEAR_RATIO) {
        return (setpoint * WHEEL_CIRCUMFERENCE_METERS) / (TICKS_PER_REV * GEAR_RATIO);
    }
      
    public static double metersPerSecToTicksPer100ms(double setpoint, double TICKS_PER_REV, double GEAR_RATIO, double WHEEL_CIRCUMFERENCE_METERS) {
        return metersToTicks(setpoint, TICKS_PER_REV, GEAR_RATIO, WHEEL_CIRCUMFERENCE_METERS) / GEAR_RATIO;
    }

    public static double ticksPer100msToMetersPerSec(double setpoint, double TICKS_PER_REV, double GEAR_RATIO, double WHEEL_CIRCUMFERENCE_METERS) {
        return ticksToMeters(setpoint, TICKS_PER_REV, GEAR_RATIO, WHEEL_CIRCUMFERENCE_METERS) * GEAR_RATIO;
    }
}