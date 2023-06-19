package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import lib.MoreMath;

import java.util.function.Supplier;


public class DefaultDriveCommand extends CommandBase {

	private final Supplier<XboxController> controller;
	private final DrivetrainSubsystem m_drive;
	private boolean slowMode = false;
	public DefaultDriveCommand(Supplier<XboxController> controller, DrivetrainSubsystem m_drive) {
		this.controller = controller;
		this.m_drive = m_drive;


		addRequirements(m_drive);
	}

	@Override
	public void initialize() {

	}

	private double deadband(double value, double deadband) {
//		if (Math.abs(value) > deadband) {
//			if (value > 0.0) {
//				return (value - deadband) / (1.0 - deadband);
//			} else {
//				return (value + deadband) / (1.0 - deadband);
//			}
//		} else {
//			return 0.0;
//		}
		if (MoreMath.minMax(value, deadband, -deadband) == deadband || MoreMath.minMax(value, deadband, -deadband) == -deadband)
		{
			return Math.pow(value, 3);

		}else{
			return 0;
		}
	}

	@Override
	public void execute() {
		if (controller.get().getPOV() != -1)
		{
			Constants.Drivetrain.DRIVE_MODE = Constants.Drivetrain.AUTO_TURN;
			m_drive.setRotateTarget(controller.get().getPOV());
		} else if (controller.get().getPOV() == -1) {
			Constants.Drivetrain.DRIVE_MODE = Constants.Drivetrain.MANUEL_DRIVE_MODE;
		}
		if (RobotState.isAutonomous() && Constants.Drivetrain.DRIVE_MODE != Constants.Drivetrain.AUTO_ALINE) {
			Constants.Drivetrain.DRIVE_MODE = Constants.Drivetrain.AUTO_DRIVE_MODE;
		}
		double leftX = deadband(controller.get().getLeftX(), .15);
		double leftY = deadband(controller.get().getLeftY(), .15);
		double rightX = deadband(controller.get().getRightX(), .15);

		if (controller.get().getLeftTriggerAxis() > .75)
		{
			if (slowMode == true)
			{
				slowMode = false;
				Constants.Drivetrain.MAX_SPEED = 3.68;
			} else if (slowMode == false) {
				Constants.Drivetrain.MAX_SPEED = 1.5;
			}
		}
		
		m_drive.drive(-leftY, -leftX, -rightX);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {

	}
}
