package lib.logging;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;

import java.util.ArrayList;

public class Logging{
	public final static Logging INSTANCE = new Logging();
	private final ArrayList<LoggerIO> logList = new ArrayList<>();
	/**
		This class is created to report errors and to log the values of pneumatics, motor and encoder values.
	 */
	public Logging()
	{
		CommandScheduler.getInstance().schedule(new RepeatCommand(new InstantCommand(this::update)));
	}

	public void addLog(LoggerIO item)
	{
		logList.add(item);
	}
	public ArrayList<LoggerIO> getList()
	{
		return logList;
	}

	/**
	 * This function updates all of the logs and needs to be called every tick.
	 */
	public void update()
	{
		for (int i = 0; i < logList.size(); i++) {
			logList.get(i).update();
		}
	}
}
