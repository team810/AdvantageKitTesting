package frc.robot.Auto;

public class PathLoader {
	private static final String[] locationList =
	{
		"NonBump",
		"Middle",
		"Bump"

	};
	private static final String[] gamePieceList =
	{
		"Cone",
		"Cube"
	};
	private static final String[] intakeTargetList =
	{
		"1",
		"2"
	};
	public static String toString(Location location, GamePeice gamePeice, IntakeTarget intakeTarget)
	{
		String path = "";
		switch (location)
		{
			case kNonBump:
				path = path.concat(locationList[0]);
				break;
			case kmiddle:
				path = path.concat(locationList[1]);
				break;
			case kBump:
				path = path.concat(locationList[2]);
				break;
		}
		path = path.concat("_");
		switch (gamePeice)
		{
			case kCone:
				path = path.concat(gamePieceList[0]);
				break;
			case kCube:
				path = path.concat(gamePieceList[1]);
				break;
		}
		path = path.concat("_");
		switch (intakeTarget)
		{
			case first:
				path = path.concat(intakeTargetList[0]);
				break;
			case second:
				path = path.concat(intakeTargetList[1]);
				break;
		}
		return path;
	}
}

enum Location
{
	kNonBump,
	kmiddle,
	kBump
}

enum GamePeice
{
	kCube,
	kCone
}

enum IntakeTarget
{
	first,
	second
}