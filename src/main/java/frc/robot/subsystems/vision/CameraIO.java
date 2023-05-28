package frc.robot.subsystems.vision;

import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

public interface CameraIO {
	public void update();
	public PhotonTrackedTarget getTarget();
	public List<PhotonTrackedTarget> getTargets();
}
