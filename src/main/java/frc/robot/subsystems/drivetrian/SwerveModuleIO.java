package frc.robot.subsystems.drivetrian;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {
	public SwerveModuleState currentState();
	public SwerveModuleState targetState();
	public void setStates(SwerveModuleState state);
	public void update();
	public void setIdleModeDrive(CANSparkMax.IdleMode mode);
	public void setIdleModeRot(CANSparkMax.IdleMode mode);
}
