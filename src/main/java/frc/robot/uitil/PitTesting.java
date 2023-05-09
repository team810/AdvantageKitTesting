package frc.robot.uitil;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PitTesting extends SequentialCommandGroup {

	public PitTesting() {
		addCommands(
                new ParallelCommandGroup(
                        ArmTesting(),
                        intakeAndConveyorTest()
                )
        );
	}
    private Command ArmTesting()
    {
        return new SequentialCommandGroup(

        );
    }
    private Command intakeAndConveyorTest()
    {
        return new SequentialCommandGroup(

        );
    }
}