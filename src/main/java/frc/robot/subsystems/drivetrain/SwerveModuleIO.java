package frc.robot.subsystems.drivetrain;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {
	public void setModule(double speed, double angle);
	public SwerveModuleState getTargetStates();
	public void setMode(CANSparkMax.IdleMode mode);
	public SwerveModulePosition getModulePosition();
	public void update();

}
