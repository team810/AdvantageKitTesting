package frc.robot.subsystems.vision;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

public class Camera {
	private final HttpCamera feed;
	private final PhotonCamera camera;

	private final String name;
	private final String prefix;

	public Camera(String name)
	{
		feed = null;

		camera = new PhotonCamera(name);

		this.name = name;

		prefix = "Vision/Raw/" + this.name;
	}
	public Camera(String name, String url)
	{

//		feed = new HttpCamera(camera1.getName(), camera1.getInfo().path);
		feed = new HttpCamera(name, url);

		camera = new PhotonCamera(name);


		this.name = name;

		prefix = "Vision/Raw/" + this.name;

	}

	public void update()
	{
		Logger.getInstance().recordOutput(prefix + "hasTargets", camera.getLatestResult().hasTargets());

		// Best Target Logging
		if (camera.getLatestResult().hasTargets())
		{
			Logger.getInstance().recordOutput(prefix + "/BestTarget/Skew", camera.getLatestResult().getBestTarget().getSkew());
			Logger.getInstance().recordOutput(prefix + "/BestTarget/Pitch", camera.getLatestResult().getBestTarget().getPitch());
			Logger.getInstance().recordOutput(prefix + "/BestTarget/Yaw", camera.getLatestResult().getBestTarget().getYaw());

			Logger.getInstance().recordOutput(prefix + "/BestTarget/Transform3d", new Pose3d(camera.getLatestResult().getBestTarget().getBestCameraToTarget().getTranslation(),new Rotation3d()));

			Logger.getInstance().recordOutput(prefix + "/BestTarget/PointX", new double[]{0.0, 15 + camera.getLatestResult().getBestTarget().getYaw()});
			Logger.getInstance().recordOutput(prefix + "/BestTarget/PointY", new double[]{0, camera.getLatestResult().getBestTarget().getPitch() + 20});

		}
	}
}
