package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenixpro.hardware.core.CoreCANcoder;
import com.ctre.phoenixpro.sim.CANcoderSimState;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;
import frc.robot.Robot;
import lib.MoreMath;
import lib.SparkMax.AdvancedIdleMode;
import lib.SparkMax.AdvancedSparkMax;
import org.littletonrobotics.junction.Logger;

public class ModuleReal implements SwerveModuleIO{
	private final CANSparkMax driveMotor;
	private final CANSparkMax steerMotor;

	private final RelativeEncoder driveEncoder;
	private final RelativeEncoder steerEncoder;

	private final SparkMaxPIDController driveController;
	private final SparkMaxPIDController steerController;

	private CANCoder canCoder;

	private final double canOffset;
	private Modules module;

	private SwerveModulePosition modulePosition;

	private double speed;
	private double angle;

	public ModuleReal(
			int driveID,
			int steerID,
			int canCoderID,
			double canCoderOffset,
			Modules module
	)
	{
		canOffset = canCoderOffset;

		driveMotor = new CANSparkMax(driveID, CANSparkMaxLowLevel.MotorType.kBrushless);
		steerMotor = new CANSparkMax(steerID, CANSparkMaxLowLevel.MotorType.kBrushless);

		driveMotor.restoreFactoryDefaults();
		steerMotor.restoreFactoryDefaults();

		driveMotor.setSmartCurrentLimit(40);
		steerMotor.setSmartCurrentLimit(20);

		canCoder = new CANCoder(canCoderID);

		CANCoderConfiguration configuration = new CANCoderConfiguration();
		configuration.magnetOffsetDegrees = canCoderOffset;
		configuration.absoluteSensorRange = AbsoluteSensorRange.valueOf(0);
		canCoder.configAllSettings(configuration);

		driveEncoder = driveMotor.getEncoder();
		steerEncoder = steerMotor.getEncoder();

		driveController = driveMotor.getPIDController();
		steerController = steerMotor.getPIDController();

		driveController.setP(6e-5);
		driveController.setI(0);
		driveController.setD(0);
		driveController.setFF(.000015);

		steerController.setP(.1);
		steerController.setI(0);
		steerController.setD(0);
		steerController.setFF(.001);

		steerController.setPositionPIDWrappingMaxInput(360);
		steerController.setPositionPIDWrappingMinInput(0);

		steerController.setPositionPIDWrappingEnabled(true);

		steerController.setOutputRange(-1,1);

		steerEncoder.setPosition(canCoder.getAbsolutePosition());

		steerController.setFeedbackDevice(steerEncoder);

		modulePosition = new SwerveModulePosition(0,new Rotation2d());
	}
	@Override
	public void setModule(double speed, double angle) // speed in motor percent and angle in degrees
	{
		this.speed = speed;
		this.angle = angle;
	}

	private void updateModule()
	{
	}
	@Override
	public SwerveModuleState getTargetStates()
	{
		return new SwerveModuleState(speed,new Rotation2d(Math.toRadians(angle)));
	}
	@Override
	public void setMode(CANSparkMax.IdleMode mode) {

	}
	@Override
	public SwerveModulePosition getModulePosition() {
		return modulePosition;
	}
	@Override
	public void update() {

	}

	private void log() {

	}
}
