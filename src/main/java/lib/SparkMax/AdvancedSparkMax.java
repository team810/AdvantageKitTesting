package lib.SparkMax;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class AdvancedSparkMax extends CANSparkMax {
	RelativeEncoder encoderSim;

	public AdvancedSparkMax(int deviceId, MotorType type) {
		super(deviceId, type);

		encoderSim = getEncoder();
	}

	public REVLibError setIdleMode(AdvancedIdleMode mode, Subsystem required) {
		switch (mode)
		{
			case kBreak:
				return super.setIdleMode(IdleMode.kBrake);
			case kCoast:
				return super.setIdleMode(IdleMode.kCoast);
			case kHardBreak:
				CommandScheduler.getInstance().schedule(new HardBreakCommand(this, required));
				return super.setIdleMode(IdleMode.kBrake);
		}
		return super.setIdleMode(IdleMode.kBrake);
	}

	public RelativeEncoder getEncoderSim()
	{
		double x = (95 *.05) * super.get();
		return encoderSim;
	}
}
