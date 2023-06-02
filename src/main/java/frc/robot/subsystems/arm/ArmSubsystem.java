package frc.robot.subsystems.arm;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    private final static ArmSubsystem INSTANCE = new ArmSubsystem();
	public static ArmSubsystem getInstance() {
		return INSTANCE;
	}

	private ArmSubsystem() {

	}
}

