package frc.robot.commands.drivetrain.alignment;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drivetrian.DrivetrainSubsystem;
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
				break;
			case kAprilTag:
				break;
			case kCube:
				break;
			case kCone:
				break;
		}

		return new InstantCommand(() -> System.out.println("HELLO"));
	}
}
