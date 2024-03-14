package frc.robot.subsystems;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelper;

public class SimpleVisionSubsystem extends SubsystemBase {
	private final DoubleSubscriber m_angleSubscription;
	// private final DoubleSubscriber m_distanceSubscription;
	private double m_angle;

	public SimpleVisionSubsystem() {
		// Setup subscription
		LimelightHelper.setPipelineIndex("limelight", 0);
		m_angleSubscription = NetworkTableInstance.getDefault().getTable("limelight")
				.getDoubleTopic("tx")
				.subscribe(0);
		// m_distanceSubscription =
		// NetworkTableInstance.getDefault().getTable("limelight")
		// .getEntry("camerapose_targetspace").getDoubleArray([6]);

	}

	@Override
	public void periodic() {
		// get the data from limelight
		m_angle = m_angleSubscription.get();
		// m_distance = m_distanceSubscription.get() - 0.5;
		// SmartDashboard.putNumber("limelight angle to turn", m_angle);
		// SmartDashboard.putNumber("limelight distance from tag", m_angle);
	}

	/**
	 * Returns the angle of the camera to the tag. Negative if the tag is left of
	 * the camera, positive if the tag is right of the camera.
	 * 
	 * @return The angle of the camera
	 */
	public double getAngle() {
		return m_angle;
	}

	/**
	 * Returns the distance of the camera to the tag.
	 * 
	 * @return The angle of the camera
	 */
	// public double getDistance() {
	// return Math.tan(m_distance / m_angle);
	// }
}
