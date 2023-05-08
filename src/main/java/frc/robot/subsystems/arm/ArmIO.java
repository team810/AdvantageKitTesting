package frc.robot.subsystems.arm;

public interface ArmIO {
	// This is going to have two instances, they are the different parts of the arm

	public void setArm(ArmState state);
	public ArmState getState();
	public void update();
}
