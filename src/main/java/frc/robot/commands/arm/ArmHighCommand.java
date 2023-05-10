package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;


public class ArmHighCommand extends CommandBase {

	public ArmHighCommand() {

		addRequirements(ArmSubsystem.getInstance());
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {

	}

	@Override
	public boolean isFinished() {

		return false;
	}

	@Override
	public void end(boolean interrupted) {

	}
}
