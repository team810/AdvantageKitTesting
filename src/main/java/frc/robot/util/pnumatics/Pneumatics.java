package frc.robot.util.pnumatics;

import edu.wpi.first.wpilibj.CompressorConfigType;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.simulation.CTREPCMSim;
import edu.wpi.first.wpilibj.simulation.REVPHSim;
import frc.robot.Robot;

public class Pneumatics {
	private static PneumaticHub rev;
	private static PneumaticsControlModule ctre;
	private static CTREPCMSim ctreSim;
	private static REVPHSim revSim;
	private static PneumaticsType type;

	private static Pneumatics INSTANCE;

	public static Pneumatics getINSTANCE() {
		if (INSTANCE == null)
		{
			INSTANCE = new Pneumatics(0,PneumaticsType.kRev);
		}
		return INSTANCE;
	}

	public Pneumatics(int port, PneumaticsType type)
	{
		switch (type)
		{
			case kRev:
				rev = new PneumaticHub(port);
				break;
			case kCtre:
				ctre = new PneumaticsControlModule(port);
				break;
		}
		revSim.setCompressorConfigType(CompressorConfigType.Digital.value);

		if(Robot.isSimulation())
		{
			switch (type)
			{
				case kRev:
					revSim = new REVPHSim(rev);
					break;
				case kCtre:
					ctreSim = new CTREPCMSim(ctre);
					break;
			}
		}

	}

	public Pneumatics(PneumaticsType type)
	{
		this(0,type);
	}


}
