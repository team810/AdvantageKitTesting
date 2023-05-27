package frc.robot.subsystems.drivetrain;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Robot;
import frc.robot.util.LoggedTunableNumber;
import lib.MoreMath;
import org.littletonrobotics.junction.Logger;

public class ModuleSim implements SwerveModuleIO {
	private final FlywheelSim driveSim;
	private final SingleJointedArmSim steerSim;

	private final double WheelDiameter = 4; // Inches
	private final double WheelCircumference = Math.PI * WheelDiameter; // Inches

	private final PIDController driveController;
	private final SlewRateLimiter driveRateController;

	private final PIDController steerController;

	private final String moduleName;

	private SwerveModuleState targetState;
	private SwerveModuleState currentState;

	private SwerveModulePosition modulePosition;

	private final static LoggedTunableNumber driveP = new LoggedTunableNumber("Drivetrain/driveP");
	private final static LoggedTunableNumber driveI = new LoggedTunableNumber("Drivetrain/driveI");;
	private final static LoggedTunableNumber driveD = new LoggedTunableNumber("Drivetrain/driveD");;

	public ModuleSim(int driveID, int steerID, int canCoderID, double canCoderOffset, Modules module)
	{
		driveSim = new FlywheelSim(
				DCMotor.getNEO(2),
				8.16,
				MoreMath.toMeters(WheelCircumference/12)
		);
		steerSim = new SingleJointedArmSim(
				DCMotor.getNEO(1),
				12.8,
				SingleJointedArmSim.estimateMOI(MoreMath.toMeters(.333333),0.1905088),
				MoreMath.toMeters(.25),
				0,
				Math.PI * 2,
				false
		);

		driveController = new PIDController(.05,.2,.00025);
		driveRateController = new SlewRateLimiter(12,-50000,0);

		steerController = new PIDController(.5,0,0);

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

		currentState = new SwerveModuleState();
		targetState = new SwerveModuleState();

		driveP.initDefault(driveController.getP());
		driveI.initDefault(driveController.getI());
		driveD.initDefault(driveController.getD());

		modulePosition = new SwerveModulePosition(0,new Rotation2d(0));
	}

	@Override
	public void setModule(double speed, double angle) {

//		speed = driveRateController.calculate(speed);

		targetState = new SwerveModuleState(speed, new Rotation2d(Math.toRadians(angle)));
		targetState = SwerveModuleState.optimize(targetState, currentState.angle);
	}

	@Override
	public SwerveModuleState getTargetStates() {
		return targetState;
	}

	@Override
	public void setMode(CANSparkMax.IdleMode mode) {

	}

	@Override
	public SwerveModulePosition getModulePosition() {
		modulePosition = new SwerveModulePosition(
				modulePosition.distanceMeters + (Robot.defaultPeriodSecs * targetState.speedMetersPerSecond),
				targetState.angle
		);
		return modulePosition;
	}

	@Override
	public SwerveModuleState getCurrentPosition() {
		return currentState;
	}

	@Override
	public void update() {

		driveSim.setInputVoltage(driveRateController.calculate(MoreMath.minMax(driveController.calculate(driveSim.getAngularVelocityRPM(),(targetState.speedMetersPerSecond * 60 * 3.15)),-1,1) * 12
		));

		steerSim.setInputVoltage(MoreMath.minMax(
				steerController.calculate(steerSim.getAngleRads(), targetState.angle.getRadians()),-1,1) * 12
		);

		driveSim.update(Robot.defaultPeriodSecs);
		steerSim.update(Robot.defaultPeriodSecs);

		currentState = new SwerveModuleState(MoreMath.toMeters((((driveSim.getAngularVelocityRPM() / 60) / 8.16) * (Math.PI * 2 * 2)) / 12),new Rotation2d(steerSim.getAngleRads()));

		Logger.getInstance().recordOutput("Drivetrain/" + moduleName + "/RPM", driveSim.getAngularVelocityRPM());
		Logger.getInstance().recordOutput("Drivetrain/" + moduleName + "/TargetRPM", targetState.speedMetersPerSecond * 60 * 3.15);
		Logger.getInstance().recordOutput("Drivetrain/" + moduleName + "/TargetAngle", Math.toDegrees(steerController.getSetpoint()));
		Logger.getInstance().recordOutput("Drivetrain/" + moduleName + "/CurrentAngle", Math.toDegrees(steerSim.getAngleRads()));
		Logger.getInstance().recordOutput("Drivetrain/" + moduleName + "/AppliedOutput", driveRateController.calculate(MoreMath.minMax(driveController.calculate(driveSim.getAngularVelocityRPM(),(targetState.speedMetersPerSecond * 60 * 3.15)),-1,1) * 12));

		Logger.getInstance().recordOutput("Drivetrain/" + moduleName + "/TargetStateSpeed", targetState.speedMetersPerSecond);
		Logger.getInstance().recordOutput("Drivetrain/" + moduleName + "/TargetStateAngle", targetState.angle.getDegrees());


		driveController.setP(driveP.get());
		driveController.setI(driveI.get());
		driveController.setD(driveD.get());

	}
}
