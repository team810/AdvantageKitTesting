package frc.robot.subsystems.drivetrian;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.Robot;
import lib.MoreMath;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
	private final CANSparkMax driveMotor;
	private final CANSparkMax steerMotor;
	private final PIDController controller;

	private CANCoder canCoder;
	private final int module;

	private SwerveModulePosition modulePosition;

	private double speed;
	private double angle;

	private double distanceTraveled;

	public SwerveModule(
			int driveID,
			int steerID,
			int canCoderID,
			double canCoderOffset,
			int module // 1 for front left 2 for front right and so 3 for back left and 4 for back right
	)
	{
		driveMotor = new CANSparkMax(driveID, CANSparkMaxLowLevel.MotorType.kBrushless);
		steerMotor = new CANSparkMax(steerID, CANSparkMaxLowLevel.MotorType.kBrushless);

		canCoder = new CANCoder(canCoderID);

		this.module = module;

		canCoder.configMagnetOffset(canCoderOffset);

		driveMotor.restoreFactoryDefaults();
		steerMotor.restoreFactoryDefaults();

		steerMotor.clearFaults();
		driveMotor.clearFaults();

//		CANCoderSimCollection sim = new CANCoderSimCollection(canCoder);

//		sim.addPosition(5);

		driveMotor.setSmartCurrentLimit(40);
		steerMotor.setSmartCurrentLimit(40);

		driveMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
		steerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

		driveMotor.setInverted(false);
		steerMotor.setInverted(false);

		driveMotor.set(0);
		steerMotor.set(0);

		controller = new PIDController(.5,0,0);

		controller.enableContinuousInput(-180,180);

		modulePosition = new SwerveModulePosition(0,new Rotation2d(0));
		shuffleBoardInit();

		distanceTraveled = 0;
	}

	public void setModule(double speed, double angle) // speed in motor percent and angle in degrees
	{
		this.speed = speed;
		this.angle = angle;


		steerMotor.set(
				MoreMath.MinMax(
						controller.calculate(canCoder.getAbsolutePosition(), angle),
						-Constants.Drivetrain.MAX_TURNING_SPEED,
						Constants.Drivetrain.MAX_TURNING_SPEED
				)
		);
		if (controller.atSetpoint())
		{
			steerMotor.set(0);
		}

		driveMotor.set(speed / Constants.Drivetrain.MAX_SPEED);

		if (Robot.isReal())
		{
			modulePosition = new SwerveModulePosition(driveMotor.get(), new Rotation2d(Math.toRadians(angle)));
		} else if (Robot.isSimulation()) {

			// This is for sim
			 modulePosition = new SwerveModulePosition(
			 modulePosition.distanceMeters + speed * .02,
				 new Rotation2d(Math.toRadians(angle))
			 );
		}
	}
	public SwerveModulePosition getModulePosition() {
		return modulePosition;
	}

	private void shuffleBoardInit() {
		String moduleName;
		switch (module) {
			case 1:
				moduleName = "Front Left";
				break;
			case 2:
				moduleName = "Front Right";
				break;
			case 3:
				moduleName = "Back left";
				break;
			case 4:
				moduleName = "Back Right";
				break;
			default:
				moduleName = "How";
				break;
		}
		ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

		ShuffleboardLayout layout = tab.getLayout(moduleName, "List");

		layout.addDouble("Raw Encoder Value", () -> canCoder.getAbsolutePosition());

		layout.addDouble("Speed", () -> driveMotor.getEncoder().getVelocity());

		layout.addDouble("Speed Set", () -> getModulePosition().distanceMeters);
		layout.addDouble("Angle", () -> getModulePosition().angle.getDegrees());

		layout.addDouble("Steer Motor Speed", () ->steerMotor.get());
		layout.addDouble("Calc", () -> controller.calculate(canCoder.getAbsolutePosition(), angle));
		layout.addBoolean("At setpoint", () -> controller.atSetpoint());
		layout.addDouble("Setpoint", () -> controller.getSetpoint());


		Logger.getInstance().recordOutput("Drivetrain/" + moduleName + "/ModuleState", new SwerveModuleState(modulePosition.distanceMeters, modulePosition.angle));

	}
}
