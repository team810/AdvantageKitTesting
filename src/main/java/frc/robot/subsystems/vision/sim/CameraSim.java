package frc.robot.subsystems.vision.sim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.vision.CameraIO;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;
import java.util.ArrayList;

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

	private ArrayList<Pose3d> posList = new ArrayList<>();

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
			feildLayout = new AprilTagFieldLayout("src/main/deploy/2023-chargedup.json");
		} catch (IOException mE) {
			throw new RuntimeException(mE);
		}
		if (DriverStation.Alliance.Red == DriverStation.getAlliance())
		{
			feildLayout.setOrigin(new Pose3d(new Pose2d(.5,.5,new Rotation2d())));
		} else if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
			feildLayout.setOrigin(new Pose3d(new Pose2d(-.5,-.5,new Rotation2d())));
		}
		visionSystem.addVisionTargets(feildLayout);
		int t =0;
		for (int i = 0; i < 6; i++) {
			for (int j = 0; j < 2; j++) {

				visionSystem.addSimVisionTarget(new SimVisionTarget(CameraSimConstants.blueCone[i][j],CameraSimConstants.TargetSize[0],CameraSimConstants.TargetSize[1], t));
				posList.add(CameraSimConstants.blueCone[i][j]);
				t++;
			}
		}

//		visionSystem.addSimVisionTarget(new SimVisionTarget(new Pose3d(0,0,0,new Rotation3d()),.5,.5,-1));
	}
	@Override
	public void update() {
		visionSystem.processFrame(DrivetrainSubsystem.getInstance().getPose());



		if (camera.getLatestResult().hasTargets())
		{
			for (int i = 0; i < camera.getLatestResult().targets.size(); i++) {
				Pose3d targetPos = new Pose3d(
						DrivetrainSubsystem.getInstance().getPose()
				);
				targetPos = targetPos.transformBy(camera.getLatestResult().getTargets().get(i).getBestCameraToTarget());
				Logger.getInstance().recordOutput("BestTargetPos_" + i, targetPos);
			}
		}
	}

	@Override
	public PhotonTrackedTarget getTarget() {
		return null;
	}
}


