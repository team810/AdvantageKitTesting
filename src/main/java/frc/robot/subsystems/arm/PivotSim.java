package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Robot;
import lib.MoreMath;
import org.littletonrobotics.junction.Logger;

public class PivotSim implements ArmIO{
	private final SingleJointedArmSim pivotSim;
	private final CANSparkMax motor;
	private final PIDController controller;
	private final double REST_POS = 152;

	private ArmState state;

	public PivotSim(int deviceID)
	{
		pivotSim = new SingleJointedArmSim(
				DCMotor.getFalcon500(1),
				25,
				0.02,
				.5,
				0,
				180,
				false
		);
		motor = new CANSparkMax(deviceID, CANSparkMaxLowLevel.MotorType.kBrushless);
		REVPhysicsSim.getInstance().addSparkMax(motor, DCMotor.getNEO(1));
		controller = new PIDController(1,1,0);
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

		double calc = MoreMath.minMax(controller.calculate(pivotSim.getAngleRads(),ArmPos.ValueForStatePivot(state)),-12,12);
		System.out.println(calc);
		pivotSim.setInputVoltage(calc);

		pivotSim.update(Robot.defaultPeriodSecs);

		Logger.getInstance().recordOutput("Voltage Draw", pivotSim.getCurrentDrawAmps());
	}

	@Override
	public double getRawPos() {
		// Angle
		return REST_POS - pivotSim.getAngleRads();
	}
}
