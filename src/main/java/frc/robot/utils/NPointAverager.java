package frc.robot.utils;

/**
 * An 'n'-point averager.
 *
 * @author Sultan Jilani
 *
 * originally from https://github.com/Team2168
 */
public class NPointAverager {

	private int averagerSize;
	private double[] averagerArray;
	private int arrayPos = 0; // Next array position to put values to be averaged
	private double lastValue = 0;

	/**
	 * Constructor for end point average class
	 * 
	 * @param n the size of N point average (>= 1)
	 */
	public NPointAverager(int n) {
		if (n < 1) {
			n = 1;
		}
		averagerSize = n;
		averagerArray = new double[averagerSize];
	}

	/**
	 * returns (gets) Average of last n values sent, as name says.
	 * 
	 * @return the Average
	 */
	public double getAverage() {
		double sum = 0;

		for (int i = 0; i < averagerSize; i++) {
			sum += averagerArray[i];
		}

		return sum / averagerSize;
	}

	public double getLatestValue() {
		return lastValue;
	}

	/**
	 * puts data in to array to be averaged, hence the class name and method name.
	 * Its like magic but cooler.
	 *
	 * @param value
	 *            the value being inserted into the array to be averaged.
	 */
	public void putData(double value) {
		lastValue = value;
		averagerArray[arrayPos] = value;
		arrayPos++;

		if (arrayPos >= averagerSize) {
			// Is equal or greater to averagorSize because array is
			// zero indexed. Rolls over index position.
			arrayPos = 0;
		}
	}

}
