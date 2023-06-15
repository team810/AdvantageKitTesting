// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Auto.Auto;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer
{
    private final XboxController secondary;

    public RobotContainer()
    {
        ArmSubsystem.getInstance();
        VisionSubsystem.getInstance();
        DrivetrainSubsystem.getInstance();

        secondary = new XboxController(0);

        // Configure the trigger bindings
        configureBindings();
    }
    
    
    /** Use this method to define your trigger->command mappings. */
    private void configureBindings()
    {
        new Trigger(() -> secondary.getAButton()).toggleOnTrue(
                new InstantCommand(() -> ArmSubsystem.getInstance().setState(ArmState.kHighCone))
        );
        new Trigger(() -> secondary.getBButton()).toggleOnTrue(
                new InstantCommand(() -> ArmSubsystem.getInstance().setState(ArmState.kRest))
        );

    }
    
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        return Auto.getInstance().generateCommand("Path");
    }
}
