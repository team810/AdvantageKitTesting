package frc.robot.subsystems.arm;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class ArmSubsystem extends SubsystemBase {

    private final static ArmSubsystem INSTANCE = new ArmSubsystem();
	public static ArmSubsystem getInstance() {
		return INSTANCE;
	}

	private final ArmIO Extender;
	private final ArmIO Pivot;

	private final ArmVisualization armVis;

	private ArmSubsystem() {
		if (Robot.isReal())
		{
			Pivot = new PivotReal(12);
			Extender = new ExtenderReal(13);
		}else{
			Pivot = new PivotSim(12);
			Extender = new ExtenderSim(13);
		}

		armVis = new ArmVisualization(Pivot, Extender);

		Pivot.setArm(ArmState.kRest);
		Extender.setArm(ArmState.kRest);
	}

	@Override
	public void periodic() {
		Pivot.update();
		Extender.update();

		armVis.update(Pivot, Extender);
	}

	public void setState(ArmState state)
	{
		Pivot.setArm(state);
		Extender.setArm(state);
		System.out.println("Set state");
	}
}

