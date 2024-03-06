import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystemAdvanced;

class PoseEstimationSubsystemTest {

	static PoseEstimationSubsystem m_pPoseEstimationSubsystem;

	@BeforeAll
	static void setup() {
		m_pPoseEstimationSubsystem = new PoseEstimationSubsystem();
		NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
		for (double x = 1; x < 1.2; x += 0.01)
			limelightTable.getEntry("botpose").setDoubleArray(new double[] { x, x, 0, 0, 0, 90 });
		for (double a = 1; a < 10; a++)
			limelightTable.getEntry("botpose").setDoubleArray(new double[] { 1.2, 1.2, 0, 0, 0, 90 + a });
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
		}
		System.out.println(m_pPoseEstimationSubsystem.estimatedPose());
	}

	@Test
	void testRotation() {
		var pose = new Pose2d(1, 0, Rotation2d.fromDegrees(0));
		assertEquals(45, PoseEstimationSubsystemAdvanced.getRotation(pose, new Translation2d(2, 1)).getDegrees(),
				0.001);
		assertEquals(90, PoseEstimationSubsystemAdvanced.getRotation(pose, new Translation2d(1, 1)).getDegrees(),
				0.001);
		assertEquals(135, PoseEstimationSubsystemAdvanced.getRotation(pose, new Translation2d(0, 1)).getDegrees(),
				0.001);
		assertEquals(-135, PoseEstimationSubsystemAdvanced.getRotation(pose, new Translation2d(0, -1)).getDegrees(),
				0.001);
		assertEquals(-90, PoseEstimationSubsystemAdvanced.getRotation(pose, new Translation2d(1, -1)).getDegrees(),
				0.001);
		assertEquals(-45, PoseEstimationSubsystemAdvanced.getRotation(pose, new Translation2d(2, -1)).getDegrees(),
				0.001);
	}

	@Test
	void testDistance() {
		var pose = new Pose2d(1, 0, Rotation2d.fromDegrees(0));
		assertEquals(Math.sqrt(2), PoseEstimationSubsystemAdvanced.getDistance(pose, new Translation2d(2, 1)), 0.001);
		assertEquals(1, PoseEstimationSubsystemAdvanced.getDistance(pose, new Translation2d(1, 1)), 0.001);
		assertEquals(Math.sqrt(2), PoseEstimationSubsystemAdvanced.getDistance(pose, new Translation2d(0, 1)), 0.001);
		assertEquals(Math.sqrt(2), PoseEstimationSubsystemAdvanced.getDistance(pose, new Translation2d(0, -1)), 0.001);
		assertEquals(1, PoseEstimationSubsystemAdvanced.getDistance(pose, new Translation2d(1, -1)), 0.001);
		assertEquals(Math.sqrt(2), PoseEstimationSubsystemAdvanced.getDistance(pose, new Translation2d(2, -1)), 0.001);
	}

	@Test
	void testTransform() {
		var pose = new Pose2d(1, 0, Rotation2d.fromDegrees(0));
		assertEquals("Transform2d(Translation2d(X: 1.00, Y: 1.00), Rotation2d(Rads: 0.79, Deg: 45.00))", "" +
				PoseEstimationSubsystemAdvanced.getTransform(pose, new Translation2d(2, 1)));
		assertEquals("Transform2d(Translation2d(X: 0.00, Y: 1.00), Rotation2d(Rads: 1.57, Deg: 90.00))", "" +
				PoseEstimationSubsystemAdvanced.getTransform(pose, new Translation2d(1, 1)));
		assertEquals("Transform2d(Translation2d(X: -1.00, Y: 1.00), Rotation2d(Rads: 2.36, Deg: 135.00))", "" +
				PoseEstimationSubsystemAdvanced.getTransform(pose, new Translation2d(0, 1)));
		assertEquals("Transform2d(Translation2d(X: -1.00, Y: -1.00), Rotation2d(Rads: -2.36, Deg: -135.00))", "" +
				PoseEstimationSubsystemAdvanced.getTransform(pose, new Translation2d(0, -1)));
		assertEquals("Transform2d(Translation2d(X: 0.00, Y: -1.00), Rotation2d(Rads: -1.57, Deg: -90.00))", "" +
				PoseEstimationSubsystemAdvanced.getTransform(pose, new Translation2d(1, -1)));
		assertEquals("Transform2d(Translation2d(X: 1.00, Y: -1.00), Rotation2d(Rads: -0.79, Deg: -45.00))", "" +
				PoseEstimationSubsystemAdvanced.getTransform(pose, new Translation2d(2, -1)));

		pose = pose.rotateBy(Rotation2d.fromDegrees(45));
		assertEquals("Transform2d(Translation2d(X: 1.12, Y: -0.71), Rotation2d(Rads: -0.56, Deg: -32.24))", "" +
				PoseEstimationSubsystemAdvanced.getTransform(pose, new Translation2d(2, 1)));
		assertEquals("Transform2d(Translation2d(X: 0.41, Y: 0.00), Rotation2d(Rads: 0.00, Deg: 0.00))", "" +
				PoseEstimationSubsystemAdvanced.getTransform(pose, new Translation2d(1, 1)));
		assertEquals("Transform2d(Translation2d(X: -0.29, Y: 0.71), Rotation2d(Rads: 1.96, Deg: 112.50))", "" +
				PoseEstimationSubsystemAdvanced.getTransform(pose, new Translation2d(0, 1)));
		assertEquals("Transform2d(Translation2d(X: -1.71, Y: -0.71), Rotation2d(Rads: -2.75, Deg: -157.50))", "" +
				PoseEstimationSubsystemAdvanced.getTransform(pose, new Translation2d(0, -1)));
		assertEquals("Transform2d(Translation2d(X: -1.00, Y: -1.41), Rotation2d(Rads: -2.19, Deg: -125.26))", "" +
				PoseEstimationSubsystemAdvanced.getTransform(pose, new Translation2d(1, -1)));
		assertEquals("Transform2d(Translation2d(X: -0.29, Y: -2.12), Rotation2d(Rads: -1.71, Deg: -97.86))", "" +
				PoseEstimationSubsystemAdvanced.getTransform(pose, new Translation2d(2, -1)));

	}

}