package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PivotSim implements ArmIO{
	private final SingleJointedArmSim pivotSim;
	private final CANSparkMax motor;
	private final PIDController controller;

	private ArmState state;

	public PivotSim(int deviceID)
	{
		pivotSim = new SingleJointedArmSim(
				DCMotor.getNEO(1),
				170,
				20,
				.5,
				0,
				90,
				false
		);
		motor = new CANSparkMax(deviceID, CANSparkMaxLowLevel.MotorType.kBrushless);
		REVPhysicsSim.getInstance().addSparkMax(motor, DCMotor.getNEO(1));
		controller = new PIDController(.01,0,0);
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
		pivotSim.setInputVoltage(controller.calculate(pivotSim.getAngleRads(),ArmPos.ValueForStatePivot(state)));
	}

	@Override
	public double getRawPos() {
		// Angle
		return pivotSim.getAngleRads();
	}
}
