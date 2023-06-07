package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import org.littletonrobotics.junction.Logger;

public class ArmVisualization {
	private ArmIO Pivot;
	private ArmIO Extender;

	private Mechanism2d armBase;
	private MechanismRoot2d robot;
	private MechanismLigament2d pivotLigament;
	private MechanismLigament2d extenderLigament;

	public ArmVisualization(ArmIO Pivot, ArmIO Extender)
	{
		this.Pivot = Pivot;
		this.Extender = Extender;

		armBase = new Mechanism2d(4,4);
		robot = armBase.getRoot("Swogged", DrivetrainSubsystem.getInstance().getPose().getX(), DrivetrainSubsystem.getInstance().getPose().getY());
		pivotLigament = new MechanismLigament2d("Pivot", 5, 0);

	}

	double x;
	public void update(ArmIO Pivot, ArmIO Extender)
	{
		this.Pivot = Pivot;
		this.Extender = Extender;
		Pose3d pivot = new Pose3d(
				.32,
				0,
				.83,
				new Rotation3d(0,Math.toRadians(Pivot.getRawPos()),0)
		);
		Logger.getInstance().recordOutput("See", pivot.getTranslation().getX());


//		Pose3d pivot = new Pose3d(
//				.32,
//				0,
//				.7,
//				new Rotation3d(0,Math.toRadians(152),0)
//		);

		Logger.getInstance().recordOutput("Arm3d", pivot);
		Logger.getInstance().recordOutput("ArmPos", Pivot.getRawPos());
	}
}
