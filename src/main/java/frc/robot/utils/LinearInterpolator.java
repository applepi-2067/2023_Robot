package frc.robot.utils;

/**
 * A class which performs linear interpolation. Can be used for joystick
 * scaling, sensor scaling, etc.
 *
 * @author james@team2168.org
 *
 * originally from https://github.com/Team2168
 */
public class LinearInterpolator {
	double[][] values;

	/**
	 * Create a new interpolator.
	 * 
	 * @param values an ascending (sorted) array of coordinate pairs
	 */
	public LinearInterpolator(double[][] values) {
		// TODO: sort the input list
		this.values = values;
	}

	/**
	 * Augment an input value using the supplied data set.
	 *
	 * @param input
	 *            the value to augment
	 * @return The adjusted value.
	 */
	public double interpolate(double input) {
		double retVal = 0.0;
		boolean done = false;
		double m, b;

		// make sure the input value falls within the range of the provided data set
		input = Util.limit(input, values[0][0], values[values.length - 1][0]);

		// Find the two points in our array, between which the input falls.
		// We will start at i = 1 since we can't have a point fall outside our
		// array.
		for (int i = 1; !done && i < values.length; i++) {
			if (input <= values[i][0]) {
				// We found where the point falls in out array, between index i
				// and i-1
				// Calculate the equation for the line. y = m*x + b
				m = Util.slope(values[i - 1][0], values[i - 1][1], values[i][0], values[i][1]);
				b = Util.intercept(m, values[i][0], values[i][1]);
				retVal = m * input + b;

				// we're finished, don't continue to loop
				done = true;
			}
		}

		return retVal;
	}
}
