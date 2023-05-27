package frc.robot.subsystems.vision;

import org.photonvision.targeting.PhotonTrackedTarget;

public interface CameraIO {

	public void update();
	public PhotonTrackedTarget getTarget();
}
