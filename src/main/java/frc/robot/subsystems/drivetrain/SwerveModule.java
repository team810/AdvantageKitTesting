package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Robot;
import lib.MoreMath;
import lib.SparkMax.AdvancedIdleMode;
import lib.SparkMax.AdvancedSparkMax;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
	private final AdvancedSparkMax driveMotor;
	private final CANSparkMax steerMotor;
	private final PIDController controller;

	private CANCoder canCoder;
	private final double canOffset;
	private final int module;

	private SwerveModulePosition modulePosition;

	private double speed;
	private double angle;

	public SwerveModule(
			int driveID,
			int steerID,
			int canCoderID,
			double canCoderOffset,
			int module // 1 for front left 2 for front right and so 3 for back left and 4 for back right
	)
	{
		driveMotor = new AdvancedSparkMax(driveID, CANSparkMaxLowLevel.MotorType.kBrushless);
		steerMotor = new CANSparkMax(steerID, CANSparkMaxLowLevel.MotorType.kBrushless);

		canCoder = new CANCoder(canCoderID);
		canOffset = canCoderOffset;


		CANCoderConfiguration configuration = new CANCoderConfiguration();
		configuration.absoluteSensorRange = AbsoluteSensorRange.valueOf(0);
		configuration.magnetOffsetDegrees = canCoderOffset;

		canCoder.configAllSettings(configuration);

		this.module = module;

		driveMotor.restoreFactoryDefaults();
		steerMotor.restoreFactoryDefaults();

		steerMotor.clearFaults();
		driveMotor.clearFaults();

		driveMotor.setSmartCurrentLimit(40);
		steerMotor.setSmartCurrentLimit(40);

		driveMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
		steerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

//		driveMotor.setIdleMode(AdvancedIdleMode.kHardBreak, DrivetrainSubsystem.getInstance());

		driveMotor.setInverted(false);
		steerMotor.setInverted(false);

		driveMotor.set(0);
		steerMotor.set(0);

		controller = new PIDController(.007,0,0);

		controller.enableContinuousInput(-180,180);

		controller.setTolerance(1);

		modulePosition = new SwerveModulePosition(0,new Rotation2d(0));
		log();
	}

	public void setModule(double speed, double angle) // speed in motor percent and angle in degrees
	{
		this.speed = speed;
		this.angle = angle;
	}

	private void updateModule()
	{
		if (Robot.isReal())
		{
			steerMotor.set(
					MoreMath.minMax(
							controller.calculate(canCoder.getAbsolutePosition(), angle),
							Constants.Drivetrain.MAX_TURNING_SPEED,
							-Constants.Drivetrain.MAX_TURNING_SPEED
					)
			);
		} else if (Robot.isSimulation()) {
			steerMotor.set(
					MoreMath.minMax(
							controller.calculate(canCoder.getAbsolutePosition(), angle),
							Constants.Drivetrain.MAX_TURNING_SPEED,
							-Constants.Drivetrain.MAX_TURNING_SPEED
					)
			);

		}
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

	public SwerveModuleState getTargetStates()
	{
		return new SwerveModuleState(speed,new Rotation2d(Math.toRadians(angle)));
	}

	public void setMode(CANSparkMax.IdleMode mode)
	{
		driveMotor.setIdleMode(mode);
		steerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
	}

	public SwerveModulePosition getModulePosition() {
		return modulePosition;
	}

	public void update()
	{
		updateModule();
		log();
	}

	private void log() {

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

		Logger.getInstance().recordOutput("Drivetrain/"+ moduleName + "/Raw Encoder Value", canCoder.getAbsolutePosition());
		Logger.getInstance().recordOutput("Drivetrain/" + moduleName + "/calc Point", Math.toDegrees(canCoder.getAbsolutePosition() + canCoder.configGetMagnetOffset()));

		Logger.getInstance().recordOutput("Drivetrain/"+ moduleName + "/Speed",driveMotor.getEncoder().getVelocity());

		Logger.getInstance().recordOutput("Drivetrain/"+ moduleName + "/Speed Set", getModulePosition().distanceMeters);
		Logger.getInstance().recordOutput("Drivetrain/"+ moduleName + "/Angle", getModulePosition().angle.getDegrees());

		Logger.getInstance().recordOutput("Drivetrain/"+ moduleName + "/Steer Motor Speed", steerMotor.get());
		Logger.getInstance().recordOutput("Drivetrain/"+ moduleName + "/Calc", controller.calculate(canCoder.getAbsolutePosition(), angle));
		Logger.getInstance().recordOutput("Drivetrain/"+ moduleName + "/At setpoint", controller.atSetpoint());
		Logger.getInstance().recordOutput("Drivetrain/"+ moduleName + "/Setpoint", controller.getSetpoint());

		Logger.getInstance().recordOutput("Drivetrain/" + moduleName + "/ModuleState", new SwerveModuleState(modulePosition.distanceMeters, modulePosition.angle));
	}
}
