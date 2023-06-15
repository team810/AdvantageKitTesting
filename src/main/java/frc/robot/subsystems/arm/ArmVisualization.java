package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class ArmVisualization {
	private ArmIO Pivot;
	private ArmIO Extender;

	private Mechanism2d armBase;
	private MechanismRoot2d robot;
	private MechanismLigament2d pivotLigament;
	private MechanismLigament2d extenderLigament;

	private LoggedTunableNumber pivotOffsetX;
	private LoggedTunableNumber pivotOffsetY;

	public ArmVisualization(ArmIO Pivot, ArmIO Extender)
	{
		this.Pivot = Pivot;
		this.Extender = Extender;

		armBase = new Mechanism2d(4,4);
		robot = armBase.getRoot("Swogged", DrivetrainSubsystem.getInstance().getPose().getX(), DrivetrainSubsystem.getInstance().getPose().getY());
		pivotLigament = new MechanismLigament2d("Pivot", 5, 0);

		pivotOffsetX = new LoggedTunableNumber("x",0);
		pivotOffsetY = new LoggedTunableNumber("y", 0);
	}
	
	public void update(ArmIO Pivot, ArmIO Extender)
	{
		this.Pivot = Pivot;
		this.Extender = Extender;
		Pose3d pivot = new Pose3d(
				.21,
				0,
				.94,
				new Rotation3d(0,Math.toRadians(Pivot.getRawPos()),0)
		);
		Translation3d translation = pivot.getTranslation();
		Pose3d extender = new Pose3d(
				translation,
				pivot.getRotation()
		);
		extender = extender.transformBy(new Transform3d(new Translation3d(pivotOffsetX.get(),0,pivotOffsetY.get()),new Rotation3d()));

		Logger.getInstance().recordOutput("Arm3d", pivot, extender);
		Logger.getInstance().recordOutput("ArmPos", Pivot.getRawPos());
	}
}
