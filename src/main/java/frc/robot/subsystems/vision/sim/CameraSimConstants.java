package frc.robot.subsystems.vision.sim;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class CameraSimConstants {
	public final static Pose3d[][] blueCone = // Starting on the bump side and going to non bump
	{
			{new Pose3d(.4 -.5,0.50 -.5, 1.25,new Rotation3d()),   new Pose3d(.71 -.5, .50 -.5, .75,new Rotation3d())},
			{new Pose3d(.4 -.5,1.60 -.5,1.25,new Rotation3d()),    new Pose3d(.71 -.5,1.6 -.5,.75,new Rotation3d())},

			{new Pose3d(.4 -.5,2.15 -.5,1.25, new Rotation3d()), new Pose3d(.70 -.5,2.15 -.5,.75,new Rotation3d())},
			{new Pose3d(.4 -.5,3.30 -.5,1.25, new Rotation3d()), new Pose3d(.70 -.5,3.30 -.5,.75,new Rotation3d())},

			{new Pose3d(.4 -.5,3.85 -.5,1.25, new Rotation3d()),  new Pose3d(.70  -.5,3.85 -.5,.75, new Rotation3d())},
			{new Pose3d(.4 -.5,4.91 -.5,1.25, new Rotation3d()),  new Pose3d(.70  -.5,4.91 -.5,.75, new Rotation3d())},
	};
	public final double DISTANCE_BETWEEN_NODES_IN_GRID = blueCone[2][0].getY() - blueCone[1][0].getY(); //
	public final double DISTANCE_BETWEEN_GAPS = blueCone[3][0].getY() - blueCone[2][0].getY();

	public final static double TargetSize[] = {.05,.05};
}
