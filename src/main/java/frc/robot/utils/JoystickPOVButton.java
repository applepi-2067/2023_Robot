package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.*;

/**
 * originally from https://github.com/Team2168
 */

/**
 * Trigger commands from an analog input on the joystick
 *
 * @author James@team2168.org
 */
public class JoystickPOVButton extends Button {

	GenericHID m_joystick;
	int m_axisNumber;
	private double THRESHOLD = 0.5;

	/**
	 * Create a button for triggering commands off a joystick's analog axis
	 *
	 * @param joystick
	 *            The GenericHID object that has the button (e.g. Joystick,
	 *            KinectStick, etc)
	 * @param axisNumber
	 *            The axis number
	 * @param threshold
	 *            The threshold to trigger above (positive) or below (negative)
	 */
	public JoystickPOVButton(GenericHID joystick, double threshold) {
		m_joystick = joystick;
		THRESHOLD = threshold;
	}

	/**
	 * Set the value above which triggers should occur (for positive thresholds) or
	 * below which triggers should occur (for negative thresholds) The default
	 * threshold value is 0.5
	 *
	 * @param threshold
	 *            the threshold value (1 to -1)
	 */
	public void setThreshold(double threshold) {
		THRESHOLD = threshold;
	}

	/**
	 * Get the defined threshold value.
	 *
	 * @return the threshold value
	 */
	public double getThreshold() {
		return THRESHOLD;
	}

	/**
	 * Get the state of the JoystickAnalogButton.
	 *
	 * @return true when the analog value exceeds the specified threshold.
	 */
	public boolean get() {
		return m_joystick.getPOV() == THRESHOLD;
	}
}