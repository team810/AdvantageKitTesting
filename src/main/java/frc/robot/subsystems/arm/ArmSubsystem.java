package frc.robot.subsystems.arm;


import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class ArmSubsystem extends SubsystemBase {

    private final static ArmSubsystem INSTANCE = new ArmSubsystem();
	public static ArmSubsystem getInstance() {
		return INSTANCE;
	}

	private final ArmIO Extender;
	private final ArmIO Pivot;

	private Mechanism2d mechanism;

	private ArmSubsystem() {
		Extender = new ExtenderSim(13);
		Pivot = new PivotSim(12);

		mechanism = new Mechanism2d(20,20);
		mechanism.getRoot("Swogged", DrivetrainSubsystem.getInstance().getPose().getX(), DrivetrainSubsystem.getInstance().getPose().getY());
	}

	@Override
	public void periodic() {
		mechanism.getRoot("Swogged", 0,0).setPosition( DrivetrainSubsystem.getInstance().getPose().getX(), DrivetrainSubsystem.getInstance().getPose().getY());
	}
}

