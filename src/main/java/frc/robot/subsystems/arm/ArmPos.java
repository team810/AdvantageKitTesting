package frc.robot.subsystems.arm;

public class ArmPos {
	public static class Extender
	{
		public final static double HighCone = 20;
		public final static double HighCube = 20;
		public final static double MidCone = 10;
		public final static double MidCube = 10;
		public final static double RestPos = 0;
	}
	public static class Pivot
	{
		public final static double HighCone = 120;
		public final static double HighCube = 20;
		public final static double MidCone = 10;
		public final static double MidCube = 10;
		public final static double RestPos = 0;
	}

	public static double ValueForStatePivot(ArmState state)
	{
		switch (state)
		{
			case kHighCube:
				return Pivot.HighCube;
			case kHighCone:
				return Pivot.HighCone;
			case kMidCube:
				return Pivot.MidCube;
			case kMidCone:
				return Pivot.MidCone;
			case kRest:
				return Pivot.RestPos;
		}
		return 0;
	}
	public static double ValueForStateExtender(ArmState state)
	{
		switch (state)
		{
			case kHighCube:
				return Extender.HighCube;
			case kHighCone:
				return Extender.HighCone;
			case kMidCube:
				return Extender.MidCube;
			case kMidCone:
				return Extender.MidCone;
			case kRest:
				return Extender.RestPos;
		}
		return 0;
	}
	
}
