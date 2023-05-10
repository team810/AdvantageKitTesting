package frc.robot.commands.drivetrain.alignment;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionTargets;

public class Alignment {
	private final DrivetrainSubsystem drivetrain;
	private final VisionSubsystem vision;
	private final VisionTargets target;

	public Alignment(DrivetrainSubsystem drivetrain, VisionSubsystem vision, VisionTargets target) {
		this.drivetrain = drivetrain;
		this.vision = vision;
		this.target = target;

		CommandScheduler.getInstance().schedule(setTarget());
	}

	private Command setTarget()
	{
		switch (target)
		{
			case kReflectiveTape:
				return null;
			case kAprilTag:
				return null;
			case kCube:
				return null;
			case kCone:
				return null;
		}
		return new InstantCommand(() -> System.out.println("idk how that is possible ??? is it null ?? why did we not get a nullprt error then ??? I am having a crisis"));
	}
}
