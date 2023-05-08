package frc.robot.subsystems.vision;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

	private static VisionSubsystem INSTANCE;

	private final Camera limelight;

	public static VisionSubsystem getInstance() {
		if (INSTANCE == null) {
			INSTANCE = new VisionSubsystem();
		}
		return INSTANCE;
	}

	private VisionSubsystem() {
		
//		limelight = new Camera("USB__Live_camera", "http://localhost:5800");

//		limelight = new Camera("Integrated_Webcam", "http://localhost:5800");

//		limelight = new Camera("limeLight", "https:10.8.10.2:5800");
		limelight = null;
//		if(RobotBase.isSimulation()) {
//			NetworkTableInstance inst = NetworkTableInstance.getDefault();
//			inst.stopServer();
//			// Change the IP address in the below function to the IP address you use to connect to the PhotonVision UI.
//			inst.startClient4("Robot Simulation");
//			inst.setServer("localhost");
//			inst.startDSClient();
//		}



	}

	@Override
    public void periodic() {
//		limelight.update();
    }
}

