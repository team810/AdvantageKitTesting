package frc.robot.subsystems.drivetrain;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Robot;
import lib.MoreMath;

public class ModuleSim implements SwerveModuleIO {
	private final FlywheelSim driveSim;
	private final FlywheelSim steerSim;

	private final double WheelDiameter = 4; // Inches
	private final double WheelCircumference = Math.PI * WheelDiameter; // Inches

	private final PIDController driveController;
	private final SlewRateLimiter driveRateController;
	private final SimpleMotorFeedforward driveFF;

	private final PIDController steerController;
	private final SimpleMotorFeedforward steerFF;

	private final String moduleName;

	public ModuleSim(int driveID, int steerID, int canCoderID, double canCoderOffset, Modules module)
	{

		driveSim = new FlywheelSim(DCMotor.getNEO(1), 8.16, MoreMath.toMeters(WheelCircumference/12));
		steerSim = new FlywheelSim(DCMotor.getNEO(1), 12.8, .025);


		driveController = new PIDController(0,0,0);
		driveRateController = new SlewRateLimiter(1);
		driveFF = new SimpleMotorFeedforward(0,0,0);

		steerController = new PIDController(0,0,0);
		steerFF = new SimpleMotorFeedforward(0,0,0);


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

	}
	@Override
	public void setModule(double speed, double angle) {

	}

	@Override
	public SwerveModuleState getTargetStates() {

	}

	@Override
	public void setMode(CANSparkMax.IdleMode mode) {

	}

	@Override
	public SwerveModulePosition getModulePosition() {

	}

	@Override
	public SwerveModuleState getCurrentPosition() {

	}

	@Override
	public void update() {

		driveSim.update(Robot.defaultPeriodSecs);
		steerSim.update(Robot.defaultPeriodSecs);
	}
}
