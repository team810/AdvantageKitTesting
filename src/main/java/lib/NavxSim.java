package lib;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;

public class NavxSim {
	private final AnalogGyro gyro;
	private final AnalogGyroSim gyroSim;

	public NavxSim(int channel)
	{
		gyro = new AnalogGyro(channel);
		gyroSim = new AnalogGyroSim(gyro);
	}

	public void update(double updateTime)
	{
		gyro.getAnalogInput();
		setAngle((getRate() * updateTime)+ getAngle());
		if (getAngle() >  2* Math.PI)
		{
			setAngle(getAngle() - 2 * Math.PI);
		} else if (getAngle() < -0) {
			setAngle(getAngle() + 2 * Math.PI);
		}
	}

	public double getAngle() {
		return gyro.getAngle();
	}

	public void setAngle(double angle) {
		gyroSim.setAngle(angle);
	}

	public double getRate() {
		return gyroSim.getRate();
	}

	public void setRate(double rate) {
		gyroSim.setRate(rate);
	}

	public void resetData() {
		gyroSim.resetData();
	}

}
