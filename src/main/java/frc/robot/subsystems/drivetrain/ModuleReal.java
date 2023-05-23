package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.Constants;
import lib.MoreMath;
import org.littletonrobotics.junction.Logger;

public class ModuleReal implements SwerveModuleIO{
	private final CANSparkMax driveMotor;
	private final CANSparkMax steerMotor;

	private final CANCoder canCoder;

	private final SlewRateLimiter driveLimeter;
	private final PIDController driveControllor;
	private final RelativeEncoder driveEncoder;

	private final PIDController steerController;

	private SwerveModuleState targetState;

	private double angleSetpoint;
	private double speedSetpoint;

	private final String moduleName;

	public ModuleReal(int driveID, int steerID, int canCoderID, double canCoderOffset, Modules module)
	{
		driveMotor = new CANSparkMax(driveID, CANSparkMaxLowLevel.MotorType.kBrushless);
		steerMotor = new CANSparkMax(steerID, CANSparkMaxLowLevel.MotorType.kBrushless);

		driveMotor.restoreFactoryDefaults();
		steerMotor.restoreFactoryDefaults();

		// Setting amps
		driveMotor.setSmartCurrentLimit(40);
		steerMotor.setSmartCurrentLimit(40);

		driveMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
		steerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);


		driveLimeter = new SlewRateLimiter(12, -12,0); // Drive limiter
		driveControllor = new PIDController(.00015,.001,0); //FIXME PID constants drive motor
		driveEncoder = driveMotor.getEncoder();


		steerController = new PIDController(.005,0,0);

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
				driveMotor.setInverted(true);
				break;
			case BL:
				moduleName = "Back Left";
				break;
			case BR:
				driveMotor.setInverted(true);
				moduleName = "Back Right";
				break;
			default:
				moduleName = "";
				break;
		}

		targetState = new SwerveModuleState();
	}
	@Override
	public void setModule(double speed, double angle) {
		SwerveModuleState temp = targetState;
		targetState = new SwerveModuleState(speed, new Rotation2d(Math.toRadians(angle)));
		targetState = SwerveModuleState.optimize(targetState, temp.angle);

		speed = speed * 3.22; // Making speed in feet

		speed = speed * 8.16; // Gear ratio
		speed = speed * 60; // changing it from per second to per minute
		
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
		speed = driveEncoder.getVelocity();
		speed = speedSetpoint / 60;
		speed = speedSetpoint / 8.16;
		speed = speedSetpoint / 3.22;

		Rotation2d angle = new Rotation2d(Math.toRadians(angleSetpoint));

		return new SwerveModuleState(speed,angle);
	}

	@Override
	public void update() {
		if (RobotState.isDisabled())
		{
			speedSetpoint = 0;
			angleSetpoint = 0;

			driveControllor.reset();
			steerController.reset();
		}

		// PID controller update
		double driveSpeed = driveControllor.calculate(driveEncoder.getVelocity(), speedSetpoint);
		double turnSpeed = steerController.calculate(canCoder.getAbsolutePosition(), angleSetpoint);

		driveSpeed = MoreMath.minMax(driveSpeed, -12, 12);
		turnSpeed = MoreMath.minMax(turnSpeed, -.75, .75);

		driveSpeed = driveLimeter.calculate(driveSpeed);

		driveMotor.setVoltage(driveSpeed);
		steerMotor.set(turnSpeed);

		// Logging
		Logger.getInstance().recordOutput("Drivetrain/" + moduleName + "/Volicty Setpoint", speedSetpoint);
		Logger.getInstance().recordOutput("Drivetrain/" + moduleName + "/Angle", canCoder.getAbsolutePosition());
		Logger.getInstance().recordOutput("Drivetrain/" + moduleName + "/AngleSetpoint", angleSetpoint );
		Logger.getInstance().recordOutput("Drivetrain/" + moduleName + "/SetSpeed",driveSpeed);
		Logger.getInstance().recordOutput("Drivetrain/" + moduleName + "/Comparing", (targetState.speedMetersPerSecond / Constants.Drivetrain.MAX_SPEED) * 5700);
		Logger.getInstance().recordOutput("Drivetrain/" + moduleName + "/Volicty", driveEncoder.getVelocity());
		Logger.getInstance().recordOutput("Drivetrain/" + moduleName + "/Angle Setpoint", angleSetpoint);
	}
}
