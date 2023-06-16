package frc.robot.Auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

import java.util.HashMap;

public class Auto {
	private static Auto instance;

	private final SwerveAutoBuilder autoBuilder;

	private final HashMap<String, Command> eventMap = new HashMap<>();

	private String autoPath;

	private Auto()
	{

		if (Robot.isSimulation())
		{
			Constants.Drivetrain.ROTATION_CONSTANTS = new PIDConstants(.25,0,0);
			Constants.Drivetrain.TRANSLATION_CONSTANTS = new PIDConstants(4,0,0);
		}

		autoBuilder = new SwerveAutoBuilder(
				DrivetrainSubsystem.getInstance()::getPose,
				DrivetrainSubsystem.getInstance()::resetPos,
				DrivetrainSubsystem.getInstance().getKinematics(),
				Constants.Drivetrain.TRANSLATION_CONSTANTS,
				Constants.Drivetrain.ROTATION_CONSTANTS,
				DrivetrainSubsystem.getInstance()::setAutoStates,
				eventMap,
				false,
				DrivetrainSubsystem.getInstance()
		);
	}

	public Command generateCommand()
	{

		String path1 = PathLoader.toString(Location.kNonBump,GamePeice.kCone, IntakeTarget.first);
		String path2 = PathLoader.toString(Location.kNonBump, GamePeice.kCube, IntakeTarget.second);

		return new SequentialCommandGroup(
				new InstantCommand(() -> Constants.Drivetrain.DRIVE_MODE = Constants.Drivetrain.AUTO_DRIVE_MODE),
				autoBuilder.followPath(PathPlanner.loadPath(path1, new PathConstraints(3.6,3.6))),
				autoBuilder.followPath(PathPlanner.loadPath(path2, new PathConstraints(3.6,3.6))),
				new InstantCommand(() -> Constants.Drivetrain.DRIVE_MODE = Constants.Drivetrain.MANUEL_DRIVE_MODE)
		);
	}


	public static Auto getInstance()
	{
		if (instance == null)
		{
			instance = new Auto();
		}
		return instance;
	}
}

