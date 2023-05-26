package frc.robot.subsystems.vision;


import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.sim.CameraSim;

public class VisionSubsystem extends SubsystemBase {

	private static VisionSubsystem INSTANCE = new VisionSubsystem();
//	private final Camera limelight;

	public static VisionSubsystem getInstance() {
		return INSTANCE;
	}
	private final CameraIO cam;
	private VisionSubsystem() {
		cam = new CameraSim("yo", 90, new Transform3d(new Translation3d(0,0,0),new Rotation3d()),2,500, 500, .01);

//		limelight = new Camera("USB__Live_camera", "http://localhost:5800");

//		limelight = new Camera("Integrated_Webcam", "http://localhost:5800");

//		limelight = new Camera("limeLight", "https:10.8.10.2:5800");
//		limelight = null;
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
		cam.update();
    }
}

