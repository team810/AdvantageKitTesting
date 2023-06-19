package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ArmControl extends SequentialCommandGroup {
	private final ArmState targetState;
	private boolean readyToDrop;
	public ArmControl(ArmState targetState)
	{
		setReadyToDrop(false);
		this.targetState = targetState;

		addCommands(
				new ArmUpCommand(targetState),
				new WaitUntilCommand(this::isReadyToDrop),
				new ArmDownCommand()
		);
	}

	public boolean isReadyToDrop() {
		return readyToDrop;
	}

	public void setReadyToDrop(boolean mReadyToDrop) {
		readyToDrop = mReadyToDrop;
	}

	private class ArmUpCommand extends CommandBase
	{
		public ArmUpCommand(ArmState targetState)
		{
			addRequirements(ArmSubsystem.getInstance());
		}

		@Override
		public void initialize() {
			super.initialize();
		}

		@Override
		public void execute() {
			super.execute();
		}

		@Override
		public void end(boolean interrupted) {
			super.end(interrupted);
		}

		@Override
		public boolean isFinished() {
			return super.isFinished();
		}
	}

	private class ArmDownCommand extends CommandBase
	{
		public ArmDownCommand()
		{
			addRequirements(ArmSubsystem.getInstance());
		}
		@Override
		public void initialize() {
			super.initialize();
		}

		@Override
		public void execute() {
			super.execute();
		}

		@Override
		public void end(boolean interrupted) {
			super.end(interrupted);
		}

		@Override
		public boolean isFinished() {
			return super.isFinished();
		}
	}
}



