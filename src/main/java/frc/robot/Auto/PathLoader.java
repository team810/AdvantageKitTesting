package frc.robot.Auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import java.util.ArrayList;

/**
 * PathLoader
 * This class is used to auto generate path trajectory's based on what the user inputs as lap data and location
 * Supports up to three different laps and is specifically made for charged up.
 *
 * This class combines a bunch of small paths to create one big path.
 */
public class PathLoader {
	private static final PathConstraints startPathConstraints = new PathConstraints(3.6, 3.6);

	// You need to keep track of the handoff point of every auto location
	/*
		Non bump side handoff cords
		3.35,4.5 ROT = 180
	 */

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

	private final ArrayList<PathPlannerTrajectory> trajectory;

	private PathLoader(
			LapData lap1,
			LapData lap2,
			LapData lap3,
			boolean lap1Filled,
			boolean lap2Filled,
			boolean lap3Filled
	)
	{
		// I know I could have just checked to see if they where null but this is quite funny
		trajectory = new ArrayList<>();

		trajectory.add(loadStartPath(lap1.getLocation()));

		if (lap1Filled && lap2Filled && lap3Filled)
		{
			trajectory.addAll(getLapTrajectory(lap1));
			trajectory.addAll(getLapTrajectory(lap2));
			trajectory.addAll(getLapTrajectory(lap3));
		} else if (lap1Filled && lap2Filled) {
			trajectory.addAll(getLapTrajectory(lap1));
			trajectory.addAll(getLapTrajectory(lap2));
		} else if (lap1Filled) {
			trajectory.addAll(getLapTrajectory(lap1));
		}
	}

	private PathPlannerTrajectory loadStartPath(Location location)
	{
		String pathName = "";
		switch (location)
		{
			case kNonBump:
				pathName = locationList[0];
				break;
			case kmiddle:
				pathName = locationList[1];
				break;
			case kBump:
				pathName = locationList[2];
				break;
		}

		pathName = pathName.concat("_Start");

		return PathPlanner.loadPath(pathName, startPathConstraints);
	}

	/**
	 *
	 * @param lap This is the data class that stores the information about the lap.
	 * @return This returns the two Path trajectory's needed for one lap
	 */
	private ArrayList<PathPlannerTrajectory> getLapTrajectory(LapData lap)
	{
		// This is just init the two string variables to prevent errors
		 String path1 = ""; // This is the intake path
		 String path2 = ""; // This is going to be the place path

		switch (lap.getLocation())
		{
			case kNonBump:
				path1 = path1.concat(locationList[0]);
				path2 = path2.concat(locationList[0]);
				break;
			case kmiddle:
				path1 = path1.concat(locationList[1]);
				path2 = path2.concat(locationList[1]);
				break;
			case kBump:
				path1 = path1.concat(locationList[2]);
				path2 = path2.concat(locationList[2]);
				break;
		}
		// This is adding the spacing between the separate parts of the file name.
		path1 = path1.concat("_");
		path2 = path2.concat("_");


		// This code is just generating a sting based on the data in lap data
		switch (lap.getIntakeLocation())
		{
			case first:
				path1 = path1.concat("intake_");
				path1 = path1.concat(intakeTargetList[0]);
				break;
			case second:
				path1 = path1.concat("intake_");
				path1 = path1.concat(intakeTargetList[1]);
				break;
		}

		path2 = path2.concat("Place_");
		switch (lap.getGamePeice()) {
			case kCube:
				path2 = path2.concat("Cube");
				break;
			case kCone:
				path2 = path2.concat("Cone");
				break;
		}


		ArrayList<PathPlannerTrajectory> trajectories = new ArrayList<PathPlannerTrajectory>();

		trajectories.add(PathPlanner.loadPath(path1,new PathConstraints(3.6,3.6)));
		trajectories.add(PathPlanner.loadPath(path2,new PathConstraints(3.6,3.6)));

		return trajectories;
	}

	public ArrayList<PathPlannerTrajectory> getTrajectory() {
		return trajectory;
	}

	/**
	 * @param lap Create a new Lap Data class and enter in what you want the constraints of the path to be. When you run get Trajectory you will get a path that follows the constraints that you determined
	 */
	public PathLoader(LapData lap)
	{
		this(lap,null,null,true,false,false);
	}
	public PathLoader(LapData lap1, LapData lap2)
	{
		this(lap1, lap2, null, true,true,false);
	}
	public PathLoader(LapData lap1, LapData lap2, LapData lap3)
	{
		this(lap1,lap2,lap3,true,true,true);
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

/**
 *  This class is designed made to store data about what you want an auto lap to be. The data is then interpreted and converted into a string that is used to load the auto Trajectory.
 */
class LapData
{
	private final GamePeice gamePeice;
	private final Location location;
	private final IntakeTarget intakeLocation;

	/**
	 * @param GamePeice // This is what game piece the robot will attempt to score
	 * @param Location This is the starting Location of the robot
	 * @param IntakeLocation This parameter determines weather the robot attempts to intake the close or far game piece in the middle of the filed
	 */
	public LapData(GamePeice GamePeice, Location Location, IntakeTarget IntakeLocation)
	{
		gamePeice = GamePeice;

		location = Location;
		intakeLocation = IntakeLocation;
	}

	public GamePeice getGamePeice() {
		return gamePeice;
	}

	public Location getLocation() {
		return location;
	}

	public IntakeTarget getIntakeLocation() {
		return intakeLocation;
	}
}