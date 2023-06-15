package frc.robot.subsystems.gripper;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;
import lib.MoreMath;

public class Gripper extends SubsystemBase {
  private GripperStates state; 
  private final CANSparkMax gripperMotor; 
  private final PIDController gripperController;
  private double gripperSetpoint;

  public Gripper() {

    this.gripperMotor = new CANSparkMax(GripperConstants.GRIPPER_ID, MotorType.kBrushless);
    this.gripperController = GripperConstants.GRIPPER_CONTROLLER; 

    this.gripperMotor.getEncoder().setPosition(0); 
    this.state = GripperStates.CLOSED; 
  }

  
  public void setOpen() {
    this.gripperMotor.set(0); // to stop it (failsafe)
    this.gripperSetpoint = 3; // value needs to be tested and found empirically 
  }

  
  public void setClosed() { 
    /* this value needs to be tested 
     * empirically in order to make sure that
     * it can actually close on a cone/cube! 
     */
    this.gripperMotor.set(-0.4); // This is the value that we currently have it at
  }

  
  public void setState(GripperStates mState) {
    this.state = mState; 
    switch (this.state) 
    {
      case OPEN: 
        setOpen(); 
        break; 
      case CLOSED:
        setClosed();
        break; 
    }
  }

  public void periodic() {
    // This method will be called once per scheduler run
    if (this.gripperMotor.getEncoder().getPosition() > 3.5) { // value for hard limiting needs to be tested as well
      gripperMotor.set(-0.25); 
    } else if (this.state == GripperStates.OPEN) {
      gripperMotor.set(MoreMath.minMax(gripperController.calculate(gripperMotor.getEncoder().getPosition(), 
                  this.gripperSetpoint), -0.2, 0.2));
    }    
  }
}
