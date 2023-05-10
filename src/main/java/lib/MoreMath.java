package lib;

public class MoreMath {
	public static double minMax(double number, double min, double max)
	{
		return Math.max(Math.min(number, max), min);
	}

}
