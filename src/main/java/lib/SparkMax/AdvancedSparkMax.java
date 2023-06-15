package lib.SparkMax;

import com.revrobotics.CANSparkMax;
import lib.logging.LoggerIO;
import lib.logging.Logging;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

public class AdvancedSparkMax extends CANSparkMax implements LoggerIO {
	private final String logLocation;
	private final Boolean logging;
	private final ArrayList<LogData> data;
	private double setPoint;

	/**
	 * Create a new object to control a SPARK MAX motor Controller
	 *
	 * @param deviceId The device ID.
	 * @param type     The motor type connected to the controller. Brushless motor wires must be connected
	 *                 to their matching colors and the hall sensor must be plugged in. Brushed motors must be
	 *                 connected to the Red and Black terminals only.
	 * @param logLocation This is the url for where in the network tables you would like the motor values to be log.
    *                       This should be it's own url and should not share a url with another motor, if it dose it will overwirte the data depending on what ever motor is called second.
	 */
	public AdvancedSparkMax(int deviceId, MotorType type, String logLocation, ArrayList<LogData> data) {
		super(deviceId, type);
		this.logLocation = logLocation;
		this.logging = true;
		this.data = data;

		Logging.INSTANCE.addLog(this);
	}
	/**
	Create a new object to control a SPARK MAX motor Controller
	 *
	 * @param deviceId The device ID.
	 * @param type     The motor type connected to the controller. Brushless motor wires must be connected
	 *                 to their matching colors and the hall sensor must be plugged in. Brushed motors must be
	 *                 connected to the Red and Black terminals only.
	 */
	public AdvancedSparkMax(int deviceId, MotorType type)
	{
		super(deviceId,type);
		this.logging = false;
		this.logLocation = "";
		this.data = new ArrayList<>();
	}

	@Override
	public void update() {
		for (int i = 0; i < data.size(); i++) {
			switch (data.get(i))
			{
				case Temperature:
					Logger.getInstance().recordOutput(logLocation + "/", this.getMotorTemperature());
					break;
				case Position:
					Logger.getInstance().recordOutput(logLocation + "/", getEncoder().getPosition());
					break;
				case Speed:
					Logger.getInstance().recordOutput(logLocation + "/", getEncoder().getVelocity());
					break;
				case Setpoint:
					Logger.getInstance().recordOutput(logLocation + "/", getSetPoint());
					break;
				default:
					throw new IllegalStateException("Unexpected value: " + data);
			}
			Logger.getInstance().recordOutput(logLocation + "/",getStickyFaults());
			Logger.getInstance().recordOutput(logLocation + "/", getFaults());
		}
	}

	public Boolean getLogging() {
		return logging;
	}

	public double getSetPoint() {
		return setPoint;
	}

	public void setSetPoint(double mSetPoint) {
		setPoint = mSetPoint;
	}
}
