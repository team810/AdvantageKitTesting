// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hardstop;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants; 

public class HardStop extends SubsystemBase {

  private final DoubleSolenoid hardStop; 
  private HardStopStates state;  

  public HardStop() {
    hardStop = Constants.PNEUMATIC_HUB.makeDoubleSolenoid(0, 7);
  }

  public void stateUpdate(HardStopStates state) {

    switch (this.state) {

      case OPEN: 
        this.state = state; 
        hardStop.set(DoubleSolenoid.Value.kForward); 
        break; 
      case CLOSED: 
        this.state = state; 
        hardStop.set(DoubleSolenoid.Value.kReverse);
        break; 
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
