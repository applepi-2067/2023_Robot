package frc.robot.utils;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Electronic braking - aka "Falcon Claw" The more the "brake" is pulled, the
 * slower output speed.
 *
 * originally from https://github.com/Team2168
 */
public class FalconClaw {
	int brakeChannel = -1;
	int speedChannel = -1;
	Joystick joystick = null;
	private double minDriveSpeed = 0.2;

	LinearInterpolator interpolator;

	// The default joystick scaling profile. This profile provides the following:
	// - Deadband around zero
	// - Large area of "fine control" - slower speeds
	// - Small area near max travel of sticks for full speed
	// The entries labeled 'minDriveSpeed' should be tweaked based on your
	// drivetrain.
	// It is the value sent to the motor controller which the drivetrain barely
	// starts moving
	private static final double joystickScale[][] = {
			/* Joystick Input, Scaled Output */
			{ 1.00, 1.00 },
			{ 0.90, 0.68 },
			{ 0.06, 0.2 }, // minDriveSpeed
			{ 0.06, 0.00 },
			{ 0.00, 0.00 },
			{ -0.06, 0.00 },
			{ -0.06, -0.2 }, // minDriveSpeed
			{ -0.90, -0.68 }, 
			{ -1.00, -1.00 } };

	/**
	 * Create a new FalconClaw with your own scaling profile.
	 * 
	 * @param joystickScale
	 *            the scaling profile. A sorted (ascending) set of input/output
	 *            pairs. Where X values are the input joystick value, and Y values
	 *            are the adjusted value
	 */
	public FalconClaw(double[][] joystickScale) {
		interpolator = new LinearInterpolator(joystickScale);
	}

	/**
	 * Create a new FalconClaw, providing josytick information for later use. Using
	 * the default joystick scaling profile
	 * 
	 * @param port
	 *            the port the joystick is on
	 * @param speedChannel
	 *            the axis to monitor for acceleration
	 * @param brakeChannel
	 *            the axis to monitor for braking
	 */
	public FalconClaw(int port, int speedChannel, int brakeChannel) {
		this(joystickScale);
		joystick = new Joystick(port);
		this.speedChannel = speedChannel;
		this.brakeChannel = brakeChannel;
	}

	/**
	 * Create a new FalconClaw, providing joystick information for later use, and
	 * your own scaling profile.
	 * 
	 * @param port
	 *            the port the joystick is on
	 * @param speedChannel
	 *            the axis to monitor for acceleration
	 * @param brakeChannel
	 *            the axis to monitor for braking
	 * @param minDriveSpeed
	 *            the value sent to the drivetrain motor controllers which begin
	 *            motion
	 * @param joystickScale
	 *            the scaling profile. A sorted (ascending) set of input/output
	 *            pairs. Where X values are the input joystick value, and Y values
	 *            are the adjusted value
	 */
	public FalconClaw(int port, int speedChannel, int brakeChannel, double minDriveSpeed, double[][] joystickScale) {
		this(joystickScale);
		joystick = new Joystick(port);
		this.speedChannel = speedChannel;
		this.brakeChannel = brakeChannel;
		this.minDriveSpeed = minDriveSpeed;
	}

	/**
	 * Create a new FalconClaw using the default scaling profile.
	 */
	public FalconClaw() {
		this(joystickScale);
	}

	/**
	 *
	 * @param inputSpeed
	 *            The input value to scale back based on brake input. (1 to -1)
	 * @param brake
	 *            The brake input value. (0 to -1)
	 * @return The adjusted value.
	 */
	public double getAdjustedValue(double inputSpeed, double brake) {
		return ((1 - ((-minDriveSpeed + 1) * Math.abs(brake))) * inputSpeed);
	}

	public double getAdjustedValue() {
		// TODO: write
		// for use when a joystick port and axis channels were provided in the
		// constructor.
		return 0.0;
	}

}
