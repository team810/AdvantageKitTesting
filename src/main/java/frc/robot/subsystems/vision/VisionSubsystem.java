package frc.robot.subsystems.vision;


import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.sim.CameraSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;

public class VisionSubsystem extends SubsystemBase {

	private static VisionSubsystem INSTANCE = new VisionSubsystem();
	public static VisionSubsystem getInstance() {
		return INSTANCE;
	}

	private List<PhotonTrackedTarget> targets = new ArrayList<>();
	private List<CameraIO> cam = new ArrayList<>();
	private VisionSubsystem() {
		cam.add(new CameraSim("cam1", 125, new Transform3d(new Translation3d(.5,.5,0),new Rotation3d()),20,500, 500, 10));
	}

	@Override
    public void periodic() {
		cam.get(0).update();
		targets.clear();

		for (int i = 0; i < cam.size(); i++) {
			CameraIO camera = cam.get(i);
			targets.addAll(camera.getTargets());
		}
		clean();

    }

	private void clean()
	{
		List<Integer> targetsViewed = new ArrayList<>();

		for (int i = 0; i < targets.size(); i++) {
			boolean found = false;
			for (int j = 0; j < targetsViewed.size(); j++) {
				if (targetsViewed.get(j) == targets.get(i).getFiducialId())
				{
					found = true;
					break;
				}
			}
			if (found == false)
			{
				targetsViewed.add(targets.get(i).getFiducialId());
			}
		}

	}

	public List<PhotonTrackedTarget> getTargets()
	{
		return targets;
	}
}

