package frc.robot.subsystems.vision;

import edu.wpi.first.cscore.HttpCamera;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

public class Camera implements CameraIO{
	private final PhotonCamera camera;
	private final HttpCamera cam;

	public Camera(String name, String url)
	{
		cam = new HttpCamera(name, url);
		camera = new PhotonCamera(name);
	}
	@Override
	public void update() {

	}

	@Override
	public PhotonTrackedTarget getTarget() {
		return camera.getLatestResult().getBestTarget();
	}

	@Override
	public List<PhotonTrackedTarget> getTargets() {
		return null;
	}
}
