package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import lib.MoreMath;

public class ExtenderSim implements ArmIO {
	private final CANSparkMax motor;
	private final PIDController controller;

	private ArmState state;

	public ExtenderSim(int DeviceID)
	{
		motor = new CANSparkMax(DeviceID, CANSparkMaxLowLevel.MotorType.kBrushless);

		REVPhysicsSim.getInstance().addSparkMax(motor, DCMotor.getNeo550(1));

		controller = new PIDController(.5,0,0);
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
		motor.setVoltage(
				MoreMath.minMax(
						controller.calculate(motor.getEncoder().getPosition(), ArmPos.ValueForStateExtender(state)),
						-12,12)
		);
	}

	@Override
	public double getRawPos() {
		return motor.getEncoder().getPosition();
	}
}
