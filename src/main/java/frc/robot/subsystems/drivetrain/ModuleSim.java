package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderSimCollection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import lib.MoreMath;
import lib.SparkMax.AdvancedSparkMax;
import org.littletonrobotics.junction.Logger;

public class ModuleSim implements SwerveModuleIO {

	private final AdvancedSparkMax drive;
	private final AdvancedSparkMax steer;
	private RelativeEncoder driveEncoder;
	private final CANCoder canCoder;
	private final CANCoderSimCollection canCoderSim;

	private final PIDController steerController;
	private final ProfiledPIDController driveController;
	private final TrapezoidProfile.Constraints constraints;

	private double speed;
	private double angle;

	private final String name;

	private final static double MAX_SPEED = 3.68808;

	private SwerveModulePosition position;

	private CANCoder driveCoderCan;

	private final CANCoderSimCollection driveCoder;

	public ModuleSim(
			int driveID,
			int steerID,
			int canCoderID,
			double canCoderOffset,
			Modules module
	)
	{
		drive = new AdvancedSparkMax(driveID, CANSparkMaxLowLevel.MotorType.kBrushless);
		steer = new AdvancedSparkMax(steerID , CANSparkMaxLowLevel.MotorType.kBrushless);
		canCoder = new CANCoder(canCoderID);

		canCoderSim = new CANCoderSimCollection(canCoder);

		constraints = new TrapezoidProfile.Constraints(5676,8000);

		driveController = new ProfiledPIDController(.006,0,0,constraints);
		driveController.setTolerance(2);

		driveCoderCan = new CANCoder(canCoderID + 5);
		driveCoder = new CANCoderSimCollection(driveCoderCan);

		steerController = new PIDController(0,0,0);

		speed = 0;
		angle = 0;

		position = new SwerveModulePosition();

		switch (module)
		{
			case FL:
				name = "Front Left";
				break;
			case FR:
				name = "Front Right";
				break;
			case BL:
				name = "Back Left";
				break;
			case BR:
				name = "Back Right";
				break;
			default:
				name = "";
				break;
		}
	}
	@Override
	public void setModule(double speed, double angle) {

		this.speed = speed;
		this.angle = angle;
	}

	@Override
	public SwerveModuleState getTargetStates() {
		return new SwerveModuleState(speed, new Rotation2d(angle));
	}

	@Override
	public void setMode(CANSparkMax.IdleMode mode) {

	}

	@Override
	public SwerveModulePosition getModulePosition() {
		return position;
	}

	@Override
	public void update() {
		driveEncoder = drive.getEncoderSim();
		canCoderSim.setVelocity((int) (steer.get() * 100));
		driveCoder.setVelocity((int) drive.get()* 100);

		double targetRPM = speed * (5576/3.6);
		drive.set(driveController.calculate(driveCoderCan.getVelocity(), targetRPM));

		Logger.getInstance().recordOutput("Drivetrain/" + name +"/TargetRPM", targetRPM);
		Logger.getInstance().recordOutput("Drivetrain/" + name + "/CurrentSpeed", driveCoderCan.getVelocity() * 3.6);
		Logger.getInstance().recordOutput("Drivetrain/" + name +"/Motor Speed", drive.get());
		Logger.getInstance().recordOutput("Drivetrain/" + name +"/At Setpoint", driveController.atSetpoint());
		Logger.getInstance().recordOutput("Drivetrain/" + name +"/Speed", speed);


	}
}
