package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
	private static IntakeSubsystem INSTANCE;
    private IntakeStates intakeState;

    private final CANSparkMax left, right;
    private double manualSpeed;

    private IntakeSubsystem() {
        left = new CANSparkMax(10, CANSparkMaxLowLevel.MotorType.kBrushless);
        right = new CANSparkMax(11, CANSparkMaxLowLevel.MotorType.kBrushless);

        this.manualSpeed = 0;
	}

    public void setManualSpeed(double manualSpeed)
    {
        this.manualSpeed = manualSpeed;
    }
    @Override
    public void periodic() {
        switch (intakeState)
        {
            case kON:
                left.set(Constants.IntakeConstants.INTAKE_SPEED);
                right.set(-Constants.IntakeConstants.INTAKE_SPEED);
                break;
            case kOF:
                left.set(0);
                right.set(0);
                break;
            case kSLOW:
                left.set(.25);
                right.set(-.25);
                break;
            case kReversedSlow:
                left.set(-.25);
                right.set(.25);
                break;
            case kReversedFast:
                left.set(-Constants.IntakeConstants.INTAKE_SPEED);
                right.set(Constants.IntakeConstants.INTAKE_SPEED);
                break;
            case kManual:
                left.set(manualSpeed);
                right.set(-manualSpeed);
                break;
        }
    }

    public IntakeStates getIntakeState() {
        return intakeState;
    }
    public void setIntakeState(IntakeStates mIntakeState) {
        intakeState = mIntakeState;
    }
    public static IntakeSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new IntakeSubsystem();
        }
        return INSTANCE;
    }
}

