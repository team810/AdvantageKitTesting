package frc.robot.Auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import java.util.ArrayList;

public class PathLoader {
	private static final PathConstraints startPathConstraints = new PathConstraints(3.6, 3.6);

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
	private ArrayList<PathPlannerTrajectory> getLapTrajectory(LapData lap)
	{
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
		path1 = path1.concat("_");
		path2 = path2.concat("_");
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

class LapData
{
	private final GamePeice gamePeice;
	private final Location location;
	private final IntakeTarget intakeLocation;

	public LapData(GamePeice mGamePeice, Location mLocation, IntakeTarget mIntakeLocation)
	{
		gamePeice = mGamePeice;

		location = mLocation;
		intakeLocation = mIntakeLocation;
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