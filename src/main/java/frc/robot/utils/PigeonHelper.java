package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

/**
 * originally from https://github.com/Team2168
 */

public class PigeonHelper extends WPI_PigeonIMU {

    private static final int SLOWEST_REPORT_SPEED_MS = 255;

    public PigeonHelper(int deviceID) {
        super(deviceID);
    }

    public PigeonHelper(TalonSRX t) {
        super(t);
    }

    public void setReducedStatusFramePeriods() {
        setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_1_General, SLOWEST_REPORT_SPEED_MS);
        setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_4_Mag, SLOWEST_REPORT_SPEED_MS);
        setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_6_Accel, SLOWEST_REPORT_SPEED_MS);
        setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_10_SixDeg_Quat, SLOWEST_REPORT_SPEED_MS);
        setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_3_GeneralAccel, SLOWEST_REPORT_SPEED_MS);
        setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR , 10);
    }
}
