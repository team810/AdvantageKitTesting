package lib.Pnumatics;

import edu.wpi.first.hal.REVPHFaults;
import edu.wpi.first.hal.REVPHStickyFaults;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.simulation.CTREPCMSim;
import edu.wpi.first.wpilibj.simulation.PneumaticsBaseSim;
import edu.wpi.first.wpilibj.simulation.REVPHSim;
import frc.robot.Robot;
import lib.logging.LoggerIO;
import org.littletonrobotics.junction.Logger;

public class Pneumatics implements LoggerIO {
	private PneumaticHub revHub;
	private PneumaticsControlModule ctreHub;
	private REVPHSim revHubSim;
	private PneumaticsBaseSim ctreHubSim;

	private final double port;
	private final PneumaticsModuleType type;
	private final String path;

	/**
	 *
	 * @param port what is the can id of the pnumatics hub
	 * @param type CTRE or REV pneumatics module
	 * @param path the network table path for logging be sure to include a back slash at the end
	 */
	public Pneumatics(int port, PneumaticsModuleType type, String path)
	{
		this.path = path;

		revHub = null;
		ctreHub = null;

		revHubSim = null;
		ctreHubSim = null;

		this.port = port;
		this.type = type;

		if (Robot.isReal())
		{
			switch (type)
			{
				case CTREPCM:
					ctreHub = new PneumaticsControlModule(port);
					break;
				case REVPH:
					revHub = new PneumaticHub(port);
					break;
			}
		}

		if (Robot.isSimulation())
		{
			switch (type)
			{
				case CTREPCM:
					ctreHubSim = new CTREPCMSim(ctreHub);
					break;
				case REVPH:
					revHubSim = new REVPHSim(revHub);
					break;
			}
		}

	}

	/**
	 * This constructor assumes that the port is 0 for the pneumatics hub.
	 * @param type CTRE or REV pneumatics module
	 * @param path the network table path for logging be sure to include a backslash at the end
	 */
	public Pneumatics(PneumaticsModuleType type, String path)
	{
		this(0,type,path);
	}

	public Compressor getCompressor()
	{
		return null;
	}

	public DoubleSolenoid createDoubleSolenoid(int fwd, int revs)
	{
		if (Robot.isReal()) {
			switch (type) {
				case REVPH:
					return revHub.makeDoubleSolenoid(fwd, revs);
				case CTREPCM:
					return ctreHub.makeDoubleSolenoid(fwd, revs);
			}
		}
		return null;
	}

	public Solenoid createSol(int channel)
	{
		if (Robot.isReal())
		{
			switch (type)
			{
				case REVPH:
					return revHub.makeSolenoid(channel);
				case CTREPCM:
					return ctreHub.makeSolenoid(channel);
			}
		}
		return null;
	}

	public boolean compressorOn()
	{
		if (Robot.isReal())
		{
			switch (type)
			{
				case CTREPCM:
					return ctreHub.getCompressor();
				case REVPH:
					return revHub.getCompressor();
			}
		} else if (Robot.isSimulation()) {
			switch (type)
			{
				case CTREPCM:
					return ctreHubSim.getCompressorOn();
				case REVPH:
					return revHubSim.getCompressorOn();
			}
		}
		return false;
	}


	// This update function is strictly for updating logging
	@Override
	public void update() {
		if (Robot.isReal())
		{
			switch (type)
			{
				case CTREPCM:
					break;
				case REVPH:
					break;
			}
		} else if (Robot.isSimulation()) {
			switch (type)
			{
				case CTREPCM:
					break;
				case REVPH:
					break;
			}
		}
	}
	private void log(boolean compressorOn, double pressure, REVPHFaults faults, REVPHStickyFaults stickyFaults)
	{
		Logger.getInstance().recordOutput(path + "Compressor_On", compressorOn);
		Logger.getInstance().recordOutput(path + "Pressure", pressure);
	}
}
