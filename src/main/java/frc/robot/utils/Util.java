package frc.robot.utils;

import java.util.Vector;

/**
 * Contains basic functions that are used often.
 *
 * Some methods originally from 254's 2013 robot code:
 * https://github.com/Team254/FRC-2013
 * /blob/master/src/com/team254/lib/util/Util.java
 *
 * @author james@team2168.org (James Corcoran)
 * @author richard@team254.com (Richard Lin)
 * @author brandon.gonzalez.451@gmail.com (Brandon Gonzalez)
 */
public class Util {
	// Prevent this class from being instantiated.
	private Util() {
	}

	/**
	 * Find the minimum of two values.
	 * 
	 * @param x
	 * @param y
	 * @return the smaller of the two values
	 */
	public static double min(double x, double y) {
		if (x > y) {
			return y;
		} else {
			return x;
		}
	}

	/**
	 * Find the maximum of two values.
	 * 
	 * @param x
	 * @param y
	 * @return the larger of the two values
	 */
	public static double max(double x, double y) {
		if (x > y) {
			return x;
		} else {
			return y;
		}
	}

	/**
	 * Coerce a number within a defined range.
	 * 
	 * @param value
	 *            the number to constrain within the defined limit
	 * @param upperLimit
	 *            number supplied value must not exceed
	 * @param lowerLimit
	 *            number supplied value must not be less than
	 * @return the supplied value limited to within the defined range
	 */
	public static double limit(double value, double lowerLimit, double upperLimit) {
		return min(max(value, lowerLimit), upperLimit);
	}

	/**
	 * Coerce a number within a range of 1.0 to -1.0
	 * 
	 * @param value
	 *            the number to constrain
	 * @return the supplied value constrained from 1.0 to -1.0
	 */
	public static double limit(double value) {
		return limit(value, -1.0, 1.0);
	}

	/**
	 * Returns the array of substrings obtained by dividing the given input string
	 * at each occurrence of the given delimiter.
	 */
	public static String[] split(String input, String delimiter) {
		Vector<String> node = new Vector<String>();
		int index = input.indexOf(delimiter);
		while (index >= 0) {
			node.addElement(input.substring(0, index));
			input = input.substring(index + delimiter.length());
			index = input.indexOf(delimiter);
		}
		node.addElement(input);

		String[] retString = new String[node.size()];
		for (int i = 0; i < node.size(); ++i) {
			retString[i] = (String) node.elementAt(i);
		}

		return retString;
	}

	/**
	 * Calculate the slope of a line given two points
	 * 
	 * @param x1
	 * @param y1
	 * @param x2
	 * @param y2
	 * @return the slope
	 */
	public static double slope(double x1, double y1, double x2, double y2) {
		return (y2 - y1) / (x2 - x1);
	}

	/**
	 * Calculate the y intercept of the line given its slope and a point on the line
	 * 
	 * @param slope
	 * @param x
	 * @param y
	 * @return the y intercept of the line.
	 */
	public static double intercept(double slope, double x, double y) {
		return y - (slope * x);
	}

	/**
	 * Get the difference in angle between two angles.
	 *
	 * @param from
	 *            The first angle
	 * @param to
	 *            The second angle
	 * @return The change in angle from the first argument necessary to line up with
	 *         the second. Always between -Pi and Pi
	 */
	public static double getDifferenceInAngleRadians(double from, double to) {
		return boundAngleNegPiToPiRadians(to - from);
	}

	/**
	 * Get the difference in angle between two angles.
	 *
	 * @param from
	 *            The first angle
	 * @param to
	 *            The second angle
	 * @return The change in angle from the first argument necessary to line up with
	 *         the second. Always between -180 and 180
	 */
	public static double getDifferenceInAngleDegrees(double from, double to) {
		return boundAngleNeg180to180Degrees(to - from);
	}

	public static double boundAngle0to360Degrees(double angle) {
		// Naive algorithm
		while (angle >= 360.0) {
			angle -= 360.0;
		}
		while (angle < 0.0) {
			angle += 360.0;
		}
		return angle;
	}

	public static double boundAngleNeg180to180Degrees(double angle) {
		// Naive algorithm
		while (angle >= 180.0) {
			angle -= 360.0;
		}
		while (angle < -180.0) {
			angle += 360.0;
		}
		return angle;
	}

	public static double boundAngle0to2PiRadians(double angle) {
		// Naive algorithm
		while (angle >= 2.0 * Math.PI) {
			angle -= 2.0 * Math.PI;
		}
		while (angle < 0.0) {
			angle += 2.0 * Math.PI;
		}
		return angle;
	}

	public static double boundAngleNegPiToPiRadians(double angle) {
		// Naive algorithm
		while (angle >= Math.PI) {
			angle -= 2.0 * Math.PI;
		}
		while (angle < -Math.PI) {
			angle += 2.0 * Math.PI;
		}
		return angle;
	}

	/**
	 * A simple low-overhead filter.
	 * 
	 * @param value
	 *            new value to add to the running average
	 * @param sum
	 *            the running total for the filtered value
	 * @param gain
	 *            a between 0.0 - 1.0. The higher the gain, the slower the sum will
	 *            respond to input changes.
	 * @return the filtered value
	 */
	public static double runningAverage(double value, double sum, double gain) {
		return (sum * gain) + (value * (1 - gain));
	}
}
