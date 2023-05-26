package frc.robot.subsystems.vision.sim;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionTargets;
import frc.robot.util.Goal;
import org.photonvision.SimVisionTarget;
import org.photonvision.targeting.TargetCorner;

import java.util.List;

public class TargetSim {

	public static SimVisionTarget generateTarget(VisionTargets type, Goal goal)
	{
		switch (type)
		{
			case kReflectiveTape:
				return limeLightTarget(goal);
			case kAprilTag:
				break;
			case kCube:
				return null;
			case kCone:
				return null;
		}
		return null;
	}

	private static SimVisionTarget limeLightTarget(Goal goal)
	{
		double yaw;
		double pitch;
		double skew;
		double id;
		Transform3d pose;
		Transform3d altPose;
		double ambiguity;
		List <TargetCorner> minAreaRectCorners;
		List <TargetCorner> detectedCorners;

		yaw = 0;
		pitch = 0;
		skew = 0;

		id = 0;
		double robotX = DrivetrainSubsystem.getInstance().getPose().getX();
		int targetSlot = 0;
		Pose3d transformPos;

//		if (goal == Goal.kHigh)
//		{
//			transformPos = new Pose3d(CameraSimConstants.blueCone[targetSlot][2]);
//		} else if (goal == Goal.kMid) {
//			transformPos = new Pose3d(CameraSimConstants.blueCone[targetSlot][1]);
//		}else{
//			transformPos = new Pose3d(CameraSimConstants.blueCone[targetSlot][1]);
//		}
//		pose = new Transform3d(new Pose3d(DrivetrainSubsystem.getInstance().getPose()),transformPos);
//		return new PhotonTrackedTarget(yaw, pitch, skew, id, pose, altPose, ambiguity, minAreaRectCorners, detectedCorners);
		return null;
	}
}
