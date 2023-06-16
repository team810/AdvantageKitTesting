package frc.robot.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.ArrayList;
import java.util.List;


public class TrajectoryCommand extends CommandBase {

	private final VisionSubsystem vision;
	private final DrivetrainSubsystem drivetrain;

	private final Trajectory trajectory;
	private final SwerveControllerCommand controller;
	private static TrajectoryConfig config = new TrajectoryConfig(4,3);;

	private final ProfiledPIDController thetaController;

	private final List<Pose2d> poseList = new ArrayList<>();

	private final PIDController translateController;


	public TrajectoryCommand(Pose2d goal)
	{
		this(List.of(goal),config);
	}
	public TrajectoryCommand(List<Pose2d> poseList)
	{
		this(poseList,config);
	}
	public TrajectoryCommand(Pose2d goal, TrajectoryConfig config)
	{
		this(List.of(goal), config);
	}
	public TrajectoryCommand(List<Pose2d> list,TrajectoryConfig xyConfig) {

		translateController = new PIDController(1,0,0);

        drivetrain = DrivetrainSubsystem.getInstance();
        vision = VisionSubsystem.getInstance();

		poseList.add(drivetrain.getPose());

		poseList.addAll(list);

		config = xyConfig;

		thetaController = new ProfiledPIDController(
				0,0,0, new TrapezoidProfile.Constraints(-Math.PI,Math.PI)
		);
		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		trajectory = TrajectoryGenerator.generateTrajectory(
			poseList,
			config
		);

		controller = new SwerveControllerCommand(
				trajectory,
				drivetrain::getPose,
				drivetrain.getKinematics(),
				translateController,
				translateController,
				thetaController,
				drivetrain::setAutoStates,
				drivetrain
		);

		addRequirements(DrivetrainSubsystem.getInstance(), VisionSubsystem.getInstance());
	}

	@Override
	public boolean isFinished() {
		return controller.isFinished();
	}
}
