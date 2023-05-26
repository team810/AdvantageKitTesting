package frc.robot.subsystems.vision.sim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.vision.CameraIO;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;

public class CameraSim implements CameraIO {
	private final String camName;
	private final double camDiagFOVDegrees;
	private final Transform3d robotToCamera;
	private final double maxLEDRangeMeters;
	private final int cameraResWidth;
	private final int cameraResHeight;
	private final double minTargetArea;

	private PhotonCamera camera;
	private SimVisionSystem visionSystem;

	public CameraSim(String CamName, double CamDiagFOVDegrees, Transform3d RobotToCamera, double MaxLEDRangeMeters, int CameraResWidth, int CameraResHeight, double MinTargetArea)
	{
		camName = CamName;
		camDiagFOVDegrees = CamDiagFOVDegrees;
		robotToCamera = RobotToCamera;
		maxLEDRangeMeters = MaxLEDRangeMeters;
		cameraResWidth = CameraResWidth;
		cameraResHeight = CameraResHeight;
		minTargetArea = MinTargetArea;

		visionSystem = new SimVisionSystem(camName, camDiagFOVDegrees, robotToCamera, MaxLEDRangeMeters, CameraResWidth, CameraResHeight, MinTargetArea);
		camera = new PhotonCamera(camName);

		AprilTagFieldLayout feildLayout;

		try {
			feildLayout = new AprilTagFieldLayout("C:\\Users\\mloma\\IdeaProjects\\AdvantageKitForkTest\\src\\main\\deploy\\2023-chargedup.json");
		} catch (IOException mE) {
			throw new RuntimeException(mE);
		}
		if (DriverStation.Alliance.Red == DriverStation.getAlliance())
		{
			feildLayout.setOrigin(new Pose3d(new Pose2d(0,0,new Rotation2d())));
		} else if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
			feildLayout.setOrigin(new Pose3d(new Pose2d(0,0,new Rotation2d())));
		}
		visionSystem.addVisionTargets(feildLayout);


		for (int i = 0; i < 6; i++) {
			for (int j = 0; j < 2; j++) {
				visionSystem.addSimVisionTarget(new SimVisionTarget(new Pose3d(CameraSimConstants.blueCone[i][j]),CameraSimConstants.TargetSize[0],CameraSimConstants.TargetSize[1], 1));
			}
		}
	}
	@Override
	public void update() {
		visionSystem.processFrame(DrivetrainSubsystem.getInstance().getPose());


		if (camera.getLatestResult().hasTargets())
		{
			Pose3d targetPos = new Pose3d(DrivetrainSubsystem.getInstance().getPose());
			targetPos.transformBy(camera.getLatestResult().getBestTarget().getBestCameraToTarget());
			Logger.getInstance().recordOutput("BestTargetPos", targetPos);
		}
	}

	@Override
	public PhotonTrackedTarget getTarget() {

		return null;
	}
}


