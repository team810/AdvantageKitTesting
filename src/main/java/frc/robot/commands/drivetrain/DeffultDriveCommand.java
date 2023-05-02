package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrian.DrivetrainSubsystem;
import lib.MoreMath;

import java.util.function.Supplier;


public class DeffultDriveCommand extends CommandBase {

	private final Supplier<XboxController> controller;
	private final DrivetrainSubsystem m_drive;

	public DeffultDriveCommand(Supplier<XboxController> controller, DrivetrainSubsystem m_drive) {
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
		if (MoreMath.MinMax(value, deadband, -deadband) == deadband || MoreMath.MinMax(value, deadband, -deadband) == -deadband)
		{
			return Math.pow(value, 3);

		}else{
			return 0;
		}
	}

	@Override
	public void execute() {
		if (RobotState.isTeleop())
		{
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

			m_drive.drive(-leftY, -leftX, rightX);
		}

		if (RobotState.isAutonomous())
		{
			m_drive.drive(0,0,0);
		}

	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {

	}
}
