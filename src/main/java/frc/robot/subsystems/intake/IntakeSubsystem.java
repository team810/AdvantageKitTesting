package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.SparkMax.SparkMaxGroup;

public class IntakeSubsystem extends SubsystemBase {
	private static IntakeSubsystem INSTANCE;
    private IntakeStates intakeState;

    private final SparkMaxGroup intakeMotors;
    private double manualSpeed;

    private IntakeSubsystem() {
        intakeMotors = new SparkMaxGroup(
                new CANSparkMax(10, CANSparkMaxLowLevel.MotorType.kBrushless),
                false,
                new CANSparkMax(11, CANSparkMaxLowLevel.MotorType.kBrushless),
                true
        );

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
                intakeMotors.set(Constants.IntakeConstants.INTAKE_SPEED);
                break;
            case kOF:
                intakeMotors.set(0);
                break;
            case kSLOW:
                intakeMotors.set(.25);
                break;
            case kReversedSlow:
                intakeMotors.set(-.25);
                break;
            case kReversedFast:
                intakeMotors.set(-Constants.IntakeConstants.INTAKE_SPEED);
                break;
            case kManual:
                intakeMotors.set(manualSpeed);
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

