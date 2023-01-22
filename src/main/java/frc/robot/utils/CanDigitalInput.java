package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.can.BaseTalon;

/**
 * originally from https://github.com/Team2168
 */

public class CanDigitalInput {

    BaseTalon motor;

    public CanDigitalInput(BaseTalon motor) {
        this.motor = motor;
    }

    public boolean isFwdLimitSwitchClosed() {
        return (this.motor.isFwdLimitSwitchClosed() == 1);
    }

    public boolean isRevLimitSwitchClosed(){
            return (this.motor.isRevLimitSwitchClosed() == 1);
    }
}
