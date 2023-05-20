package frc.robot;

import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;

public class Constants {

	public static final boolean tuningMode = true;
	public static final XboxController driveController = new XboxController(0);

	public static final PneumaticHub PNEUMATIC_HUB = new PneumaticHub(18); 

	public static final class Drivetrain
	{
		public static PIDConstants TRANSLATION_CONSTANTS = new PIDConstants(1,0,0);
		public static PIDConstants ROTATION_CONSTANTS = new PIDConstants(1,0,0);

		public static final double TURNING_SPEED_SIM = .7; // Degrees per tick

		public static final double MAX_TURNING_SPEED = .7;
		public static final class frontLeft
		{
			public final static int STEER_ID = 5;
			public final static int DRIVE_ID = 6;

			public final static int CAN_CODER_ID = 15;
			public final static double CAN_CODER_OFFSET = 269.30;
		}
		public static final class frontRight
		{
			public final static int STEER_ID = 7;
			public final static int DRIVE_ID = 8;

			public final static int CAN_CODER_ID = 13;
			public final static double CAN_CODER_OFFSET = 238.62;
		}
		public static final class backLeft
		{
			public final static int STEER_ID = 1;
			public final static int DRIVE_ID = 2;

			public final static int CAN_CODER_ID = 16;
			public final static double CAN_CODER_OFFSET = 168.22;
		}
		public static final class backRight
		{
			public final static int STEER_ID = 3;
			public final static int DRIVE_ID = 4;

			public final static int CAN_CODER_ID = 14;
			public final static double CAN_CODER_OFFSET = 79.63;
		}

		public static final int MANUEL_DRIVE_MODE = 1;
		public static final int AUTO_DRIVE_MODE = 2;
		public static final int AUTO_ALINE = 3;
		public static final int AUTO_TURN = 4; // AUTO turn will still allow x and y movement
		public static final int DISABLED = 5;
		public static int DRIVE_MODE = 0;

		public static double MAX_SPEED = 3.68;

		public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.635;
		public static final double DRIVETRAIN_WHEELBASE_METERS = 0.635;
		public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
				// Front left
				new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
						DRIVETRAIN_WHEELBASE_METERS / 2.0),
				// Front right
				new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
						-DRIVETRAIN_WHEELBASE_METERS / 2.0),
				// Back left
				new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
						DRIVETRAIN_WHEELBASE_METERS / 2.0),
				// Back right
				new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
						-DRIVETRAIN_WHEELBASE_METERS / 2.0));
	}

	public static class IntakeConstants
	{
		public static final double INTAKE_SPEED = .8;
	}

	public static final class GripperConstants
	{
		public static final int GRIPPER_ID = 14; 
		public static final PIDController GRIPPER_CONTROLLER = new PIDController(0.15, 0, 0);
	}
}
