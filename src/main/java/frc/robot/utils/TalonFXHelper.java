package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/**
 * originally from https://github.com/Team2168
 */

public class TalonFXHelper extends WPI_TalonFX {

    private static final int SLOWEST_REPORT_SPEED_MS = 255;

    public TalonFXHelper(int deviceID) {
        super(deviceID);
    }

    /**
     * Set all status frames to slowest report rate.
     */
    public void configFollowerStatusFrameRates() {
        setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, SLOWEST_REPORT_SPEED_MS);
        setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, SLOWEST_REPORT_SPEED_MS);
        setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, SLOWEST_REPORT_SPEED_MS);
        setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, SLOWEST_REPORT_SPEED_MS);
        setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, SLOWEST_REPORT_SPEED_MS);
        setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, SLOWEST_REPORT_SPEED_MS);
        setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, SLOWEST_REPORT_SPEED_MS);
        setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, SLOWEST_REPORT_SPEED_MS);
        setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, SLOWEST_REPORT_SPEED_MS);
        setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, SLOWEST_REPORT_SPEED_MS);
        setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, SLOWEST_REPORT_SPEED_MS);
        setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, SLOWEST_REPORT_SPEED_MS);
        setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, SLOWEST_REPORT_SPEED_MS);
        setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, SLOWEST_REPORT_SPEED_MS);
    }

    /**
     * Set status frame rates for motors operating in percent output modes
     */
    public void configOpenLoopStatusFrameRates() {
        // setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, SLOWEST_REPORT_SPEED_MS); // leave this if there's 
        setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, SLOWEST_REPORT_SPEED_MS);
        setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, SLOWEST_REPORT_SPEED_MS);
        setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, SLOWEST_REPORT_SPEED_MS);
        setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, SLOWEST_REPORT_SPEED_MS);
        setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, SLOWEST_REPORT_SPEED_MS);
        setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, SLOWEST_REPORT_SPEED_MS);
        setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, SLOWEST_REPORT_SPEED_MS);
        setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, SLOWEST_REPORT_SPEED_MS);
        setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, SLOWEST_REPORT_SPEED_MS);
        setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, SLOWEST_REPORT_SPEED_MS);
        setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, SLOWEST_REPORT_SPEED_MS);
        setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, SLOWEST_REPORT_SPEED_MS);
        setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, SLOWEST_REPORT_SPEED_MS);
    }

    public void configClosedLoopStatusFrameRates() {
        // setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, SLOWEST_REPORT_SPEED_MS);
        // setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, SLOWEST_REPORT_SPEED_MS);
        setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, SLOWEST_REPORT_SPEED_MS);
        setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, SLOWEST_REPORT_SPEED_MS);
        setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, SLOWEST_REPORT_SPEED_MS);
        setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, SLOWEST_REPORT_SPEED_MS);
        setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, SLOWEST_REPORT_SPEED_MS);
        setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, SLOWEST_REPORT_SPEED_MS);
        // setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, SLOWEST_REPORT_SPEED_MS);
        // setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, SLOWEST_REPORT_SPEED_MS);
        // setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, SLOWEST_REPORT_SPEED_MS);
        // setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, SLOWEST_REPORT_SPEED_MS);
        setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, SLOWEST_REPORT_SPEED_MS);
        setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, SLOWEST_REPORT_SPEED_MS);
    }
}
