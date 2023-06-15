package frc.robot.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;


public class TrajectoryCommand extends CommandBase {
//    private final Trajectory trajectory;

    private final VisionSubsystem vision;
    private final DrivetrainSubsystem drivetrain;
	public TrajectoryCommand(Pose2d goal) {

        drivetrain = DrivetrainSubsystem.getInstance();
        vision = VisionSubsystem.getInstance();

		addRequirements(DrivetrainSubsystem.getInstance(), VisionSubsystem.getInstance());
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {

	}

	@Override
	public boolean isFinished() {
		// TODO: Make this return true when this Command no longer needs to run execute()
		return false;
	}

	@Override
	public void end(boolean interrupted) {

	}
}
