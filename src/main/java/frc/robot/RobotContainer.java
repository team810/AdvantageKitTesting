// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrian.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer
{

    public RobotContainer()
    {
        VisionSubsystem.getInstance();
        DrivetrainSubsystem.getInstance();


        // Configure the trigger bindings
        configureBindings();
    }
    
    
    /** Use this method to define your trigger->command mappings. */
    private void configureBindings()
    {

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
