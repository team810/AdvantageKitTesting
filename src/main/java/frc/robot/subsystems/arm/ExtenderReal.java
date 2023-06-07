package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;

public class ExtenderReal implements ArmIO {
	private final CANSparkMax motor;
	private final RelativeEncoder encoder;
	private final PIDController controller;

	private ArmState state;

	public ExtenderReal(int deviceID)
	{
		motor = new CANSparkMax(deviceID, CANSparkMaxLowLevel.MotorType.kBrushless);
		encoder = motor.getEncoder();

		controller = new PIDController(0,0,0);
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

	}

	@Override
	public double getRawPos() {
		return 0;
	}
}
