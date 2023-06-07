package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Robot;
import lib.MoreMath;

public class PivotSim implements ArmIO{
	private final SingleJointedArmSim pivotSim;
	private final CANSparkMax motor;
	private final PIDController controller;
	private final double REST_POS = 152;

	private ArmState state;

	public PivotSim(int deviceID)
	{
		pivotSim = new SingleJointedArmSim(
				DCMotor.getNEO(1).withReduction(170),
				1,
				.025,
				.5,
				0,
				Math.toRadians(180),
				true
		);
		motor = new CANSparkMax(deviceID, CANSparkMaxLowLevel.MotorType.kBrushless);
		REVPhysicsSim.getInstance().addSparkMax(motor, DCMotor.getNEO(1));
		controller = new PIDController(8,0,0);
	}
	@Override
	public void setArm(ArmState state) {
		this.state = state;
	}

	@Override
	public ArmState getState() {
		return state;
	}

	@Override
	public void update() {

		double calc = MoreMath.minMax(controller.calculate(pivotSim.getAngleRads(), Math.toRadians(ArmPos.ValueForStatePivot(state))),-9,9);

		pivotSim.setInputVoltage(calc);

		pivotSim.update(Robot.defaultPeriodSecs);
	}

	@Override
	public double getRawPos() {
		// Angle
		return REST_POS - Math.toDegrees(pivotSim.getAngleRads());
	}
}
