package frc.robot.subsystems.vision.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class CameraSimConstants {
	public final static Pose2d[][] blueCone = // Starting on the bump side and going to non bump
	{
			{new Pose2d(.4,.5, new Rotation2d()), new Pose2d(.71,.5, new Rotation2d())},
			{new Pose2d(.4,1.6,new Rotation2d()), new Pose2d(.71, 1.6, new Rotation2d())},

			{new Pose2d(.4, 2.15, new Rotation2d()), new Pose2d(.7, 2.15, new Rotation2d())},
			{new Pose2d(.4, 3.30, new Rotation2d()), new Pose2d(.7, 3.30, new Rotation2d())},

			{new Pose2d(.4,3.85, new Rotation2d()), new Pose2d(.7, 3.85, new Rotation2d())},
			{new Pose2d(.4,4.91, new Rotation2d()), new Pose2d(.7,4.91, new Rotation2d())},
	};
	public final double DISTANCE_BETWEEN_NODES_IN_GRID = blueCone[2][0].getY() - blueCone[1][0].getY(); //
	public final double DISTANCE_BETWEEN_GAPS = blueCone[3][0].getY() - blueCone[2][0].getY();

	public final static double TargetSize[] = {.05,.05};
}
