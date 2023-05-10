package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class Extender implements ArmIO {
	private ArmState state;

	private final CANSparkMax motor;

	public Extender()
	{
		motor = new CANSparkMax(12, CANSparkMaxLowLevel.MotorType.kBrushless);

		motor.restoreFactoryDefaults();

		motor.setSmartCurrentLimit(20);
		motor.setIdleMode(CANSparkMax.IdleMode.kBrake);

	}

	public void motorUpdate()
	{

	}
	@Override
	public void update() {
		motorUpdate();
		log();
	}
	private void log()
	{

	}

	@Override
	public void setArm(ArmState state) {
		this.state = state;
	}

	@Override
	public ArmState getState() {
		return state;
	}
}
