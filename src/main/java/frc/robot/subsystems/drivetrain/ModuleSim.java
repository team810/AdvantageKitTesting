package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import lib.MoreMath;
import org.littletonrobotics.junction.Logger;

public class ModuleSim implements SwerveModuleIO {
	private final CANSparkMax driveMotor;
	private final CANSparkMax steerMotor;

	private final CANCoder canCoder;

	private final SlewRateLimiter driveLimeter;
	private final PIDController driveControllor;
	private final RelativeEncoder driveEncoder;
	private final SimpleMotorFeedforward driveFF;

	private final ProfiledPIDController steerController;

	private SwerveModuleState targetState;

	private double angleSetpoint;
	private double speedSetpoint;

	private final String moduleName;

	public ModuleSim(int driveID, int steerID, int canCoderID, double canCoderOffset, Modules module)
	{
		driveMotor = new CANSparkMax(driveID, CANSparkMaxLowLevel.MotorType.kBrushless);
		steerMotor = new CANSparkMax(steerID, CANSparkMaxLowLevel.MotorType.kBrushless);

		driveMotor.restoreFactoryDefaults();
		steerMotor.restoreFactoryDefaults();

		// Setting amps
		driveMotor.setSmartCurrentLimit(40);
		steerMotor.setSmartCurrentLimit(20);

		driveMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
		steerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

		driveLimeter = new SlewRateLimiter(5700, -50000,0);
//		driveLimeter = new SlewRateLimiter(5700); // This is going to be in rpm
		driveControllor = new PIDController(0,0,0); //FIXME PID constants drive motor
		driveEncoder = driveMotor.getEncoder();

		driveFF = new SimpleMotorFeedforward(0,0,0); // FIXME FF constants

		steerController = new ProfiledPIDController(0,0,0,new TrapezoidProfile.Constraints(0,0)); // FIXME pid constants steer motor

		steerController.enableContinuousInput(0,360);
		steerController.setTolerance(1);

		canCoder = new CANCoder(canCoderID);

		CANCoderConfiguration config = new CANCoderConfiguration();
		config.absoluteSensorRange = AbsoluteSensorRange.valueOf(1);
		config.magnetOffsetDegrees = canCoderOffset;

		canCoder.configAllSettings(config);

		switch (module)
		{
			case FL:
				moduleName = "Front Left";
				break;
			case FR:
				moduleName = "Front Right";
				break;
			case BL:
				moduleName = "Back Left";
				break;
			case BR:
				moduleName = "Back Right";
				break;
			default:
				moduleName = "Front Left";
				break;
		}

		targetState = new SwerveModuleState();
	}
	@Override
	public void setModule(double speed, double angle) {

		SwerveModuleState temp = targetState;
		targetState = new SwerveModuleState(speed, new Rotation2d(Math.toRadians(angle)));
//		targetState = SwerveModuleState.optimize(targetState, temp.angle);

		speed = speed * 3.22; // Making speed in feet

		speed = speed * 8.16; // Gear ratio
		speed = speed * 60; // changing it from per second to per minute

		speed = driveLimeter.calculate(speed);
		MoreMath.minMax(speed, -5700, 5700);
		speedSetpoint = speed;
		angleSetpoint = targetState.angle.getDegrees();

	}

	@Override
	public SwerveModuleState getTargetStates() {
		return targetState;
	}

	@Override
	public void setMode(CANSparkMax.IdleMode mode) {
		driveMotor.setIdleMode(mode);
	}

	@Override
	public SwerveModulePosition getModulePosition() {
		return new SwerveModulePosition(driveEncoder.getPosition(),new Rotation2d(Math.toRadians(canCoder.getAbsolutePosition())));
	}

	@Override
	public SwerveModuleState getCurrentPosition() {
		setModule(targetState.speedMetersPerSecond, targetState.angle.getDegrees());
		double speed;
		speed = speedSetpoint;
		speed = speedSetpoint / 60;
		speed = speedSetpoint / 8.16;
		speed = speedSetpoint / 3.22;

		Rotation2d angle = new Rotation2d(Math.toRadians(angleSetpoint));

		return new SwerveModuleState(speed,angle);
	}

	@Override
	public void update() {

		// PID controller update
		double driveSpeed = driveControllor.calculate(driveEncoder.getVelocity(), speedSetpoint);
		double turnSpeed = steerController.calculate(canCoder.getAbsolutePosition(), angleSetpoint);

		driveSpeed = MoreMath.minMax(driveSpeed, -1, 1);
		turnSpeed = MoreMath.minMax(turnSpeed, -.75, .75);

		driveMotor.set(driveSpeed);
		steerMotor.set(turnSpeed);

		// Logging
		Logger.getInstance().recordOutput("Drivetrain/" + moduleName + "/Volicty Setpoint", speedSetpoint);
		Logger.getInstance().recordOutput("Drivetrain/" + moduleName + "/SetSpeed",driveSpeed);
//		Logger.getInstance().recordOutput("Drivetrain/" + moduleName + "/Comparing", (targetState.speedMetersPerSecond / Constants.Drivetrain.MAX_SPEED) * 5700);

		Logger.getInstance().recordOutput("Drivetrain/" + moduleName + "/Angle Setpoint", angleSetpoint);
	}
}
