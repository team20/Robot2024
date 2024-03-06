package frc.robot.subsystems;

import static frc.robot.Constants.PoseConstants.*;

import java.util.EnumSet;
import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The purpose of the {@code LimeLightSubsystem} is to provide the pose of the
 * robot, estimated based on a variety of sources including LimeLight as well as
 * encoders and sensors attached to the robot.
 * 
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 */
public class LimeLightSubsystem extends SubsystemBase {

	/**
	 * A subclass of {@code Pose2d} for simple construction and string
	 * representation of {@code Pose2d} instances.
	 */
	public static class Pose extends Pose2d {

		/**
		 * Constrcuts a {@code Pose}
		 * 
		 * @param x            the x-coordinate value
		 * @param y            the y-coordinate value
		 * @param yawInDegrees the yaw in degrees
		 */
		public Pose(double x, double y, double yawInDegrees) {
			super(x, y, Rotation2d.fromDegrees(yawInDegrees));
		}

		/**
		 * Constrcuts a copy of the specified {@code Pose2d}
		 * 
		 * @param pose a {@code Pose2d}
		 */
		public Pose(Pose2d pose) {
			super(pose.getX(), pose.getY(), pose.getRotation());
		}

		/**
		 * Returns a string representation of this {@code Pose}.
		 */
		@Override
		public String toString() {
			return String.format("[%.1f, %.1f, %.1f degrees]", getX(), getY(), getRotation().getDegrees());
		}

		/**
		 * Returns the sum of this {@code Pose} and the specified {@code Pose}.
		 * 
		 * @param other a {@code Pose}
		 * @return the sum of this {@code Pose} and the specified {@code Pose}
		 */
		public Pose2d add(Pose other) {
			return new Pose2d(getTranslation().plus(other.getTranslation()), getRotation().plus(other.getRotation()));
		}

		/**
		 * Returns the average of the specified {@code Pose2d}s.
		 * 
		 * @param poses
		 *              {@code Pose}s
		 * @return the average of the specified {@code Pose2d}s
		 */
		public static Pose2d average(Pose2d... poses) {
			if (poses == null || poses.length == 0)
				return null;
			double x = 0;
			double y = 0;
			double yaw = 0;
			int count = 0;
			for (var pose : poses) {
				if (pose != null) {
					x += pose.getX();
					y += pose.getY();
					yaw += pose.getRotation().getDegrees();
					count++;
				}
			}
			if (count == 0)
				return null;
			return new Pose2d(x / poses.length, y / poses.length, Rotation2d.fromDegrees(yaw / poses.length));
		}

	}

	/**
	 * A {@code Pose2d} representing the center of the field.
	 */
	protected static final Pose2d DEFAULT_POSE = new Pose(0, 0, 0);

	/**
	 * The most recent botpose data obtained from LimeLight.
	 */
	protected double[] m_botpose;

	/**
	 * This {@code LimeLightSubsystem}'s confidence about its estimated pose.
	 */
	protected double m_confidence = 0;

	/**
	 * Constructs a {@code LimeLightSubsystem}.
	 */
	public LimeLightSubsystem() {
		subscribe("limelight", "botpose", new double[6], event -> changedBotPose(event));
	}

	/**
	 * Returns a {@code Pose2d} representing the estimated pose of the robot.
	 * 
	 * @return a {@code Pose2d} representing the estimated pose of the robot
	 *         ({@code null} if it has not been possible to reliably estimate the
	 *         pose of the robot)
	 */
	public Pose2d estimatedPose() {
		if (m_botpose == null)
			return null;
		var pose = new Pose(m_botpose[0], m_botpose[1], m_botpose[5]);
		return pose.equals(DEFAULT_POSE) ? null : pose;
	}

	/**
	 * Returns this {@code LimeLightSubsystem}'s confidence about its estimated
	 * pose.
	 * 
	 * @return a value between 0 (no confidence at all) and 1 (absolute confidence)
	 */
	public double confidence() {
		return m_confidence;
	}

	/**
	 * Adds the specified {@code Listener} to respond to the changes in the
	 * specified topic in the specified table.
	 * 
	 * @param tableName    the name of the table
	 * @param topicName    the name of the topic
	 * @param defaultValue the default value used when no value was obtained
	 *                     regarding the topic
	 * @param listener     a {@code Consumer} responsible for responding to the
	 *                     changes in the specified topic
	 */
	public static void subscribe(String tableName, String topicName, double[] defaultValue,
			Consumer<NetworkTableEvent> listener) {
		NetworkTableInstance i = NetworkTableInstance.getDefault();
		NetworkTable t = i.getTable(tableName);
		var s = t.getDoubleArrayTopic(topicName).subscribe(defaultValue);
		i.addListener(s, EnumSet.of(NetworkTableEvent.Kind.kValueAll), listener);
	}

	/**
	 * Is invoked whenever the "botpose" entry in the "limelight" table changes.
	 * 
	 * @param event a {@code NetworkTableEvent} regarding the change in the
	 *              "botpose" entry in the "limelight" table
	 * @return a {@code double} array from the "botpose" entry in the "limelight"
	 *         table
	 */
	protected double[] changedBotPose(NetworkTableEvent event) {
		boolean validSample = false;
		try {
			var v = event.valueData.value;
			m_botpose = v.getDoubleArray();
			if (Math.abs(m_botpose[0]) > 0.1 || Math.abs(m_botpose[1]) > 0.1)
				validSample = true;
		} catch (Exception e) {
			m_botpose = null;
			// e.printStackTrace();
		} finally {
			m_confidence = 0.96 * m_confidence + (validSample ? 0.04 : 0);
		}
		return m_botpose;
	}

	/**
	 * Returns the transformation from the estimated pose of of the robot to the
	 * specified target {@code Pose}.
	 * 
	 * @param targetPose the target {@code Pose}
	 * @return the transformation from the estimated pose of of the robot to the
	 *         specified target {@code Pose}; {@code null} if it has not been
	 *         possible to reliably estimate the pose of the robot
	 */
	public Transform2d transformationTo(Pose2d targetPose) {
		var pose = estimatedPose();
		if (pose == null)
			return null;
		return targetPose.minus(pose);
	}

	/**
	 * Returns the transformation needed for the robot to face toward the specified
	 * target position
	 * 
	 * @param targetPosition the target position
	 * @return the transformation needed for the robot to face toward the specified
	 *         target
	 *         position; {@code null} if it has not been
	 *         possible to reliably estimate the pose of the robot
	 */
	public Transform2d transformationToward(Translation2d targetPosition) {
		var pose = estimatedPose();
		if (pose == null)
			return null;
		Translation2d diff = targetPosition.minus(pose.getTranslation());
		if (diff.getNorm() == 0)
			return null;
		var targetPose = new Pose2d(pose.getTranslation(), diff.getAngle());
		return targetPose.minus(pose);
	}

	/**
	 * Returns the transformation needed for the robot to face toward the specified
	 * target position and remain the specified distance away fron the target
	 * position.
	 * 
	 * @param targetPosition   the target position whose x and y-coordinate values
	 *                         are in meters
	 * @param distanceToTarget the desired distance in meters to the target
	 * @return the transformation needed for the robot to face toward the specified
	 *         target position and remain the specified distance away fron the
	 *         target position; {@code null} if it has not been
	 *         possible to reliably estimate the pose of the robot
	 */
	public Transform2d transformationToward(Translation2d targetPosition, double distanceToTarget) {
		var pose = estimatedPose();
		if (pose == null)
			return null;
		Translation2d diff = targetPosition.minus(pose.getTranslation());
		if (diff.getNorm() == 0)
			return null;
		var targetPose = new Pose2d(pose.getTranslation().plus(diff.times(1 - distanceToTarget / diff.getNorm())),
				diff.getAngle());
		return targetPose.minus(pose);
	}

	/**
	 * Calculates the distance in meters to the target position.
	 * 
	 * @param targetPosition the target position
	 * @return the distance to the target position; {@code null} if it has not been
	 *         possible to reliably estimate the pose of the robot
	 */
	public Double distanceTo(Translation2d targetPosition) {
		var t = transformationToward(targetPosition, 0);
		if (t == null)
			return null;
		return t.getTranslation().getNorm();
	}

	/**
	 * Calculates the rotation angle in degrees to the target position.
	 * 
	 * @param targetPosition the target position
	 * @return the rotation angle in degrees to the target position; {@code null} if
	 *         it has not been
	 *         possible to reliably estimate the pose of the robot
	 */
	public Double angleTo(Translation2d targetPosition) {
		var t = transformationToward(targetPosition, 0);
		if (t == null)
			return null;
		return t.getRotation().getDegrees();
	}

	/**
	 * Calculates the distance in meters to the speaker.
	 * 
	 * @return the distance to the speaker; ; {@code null} if
	 *         it has not been
	 *         possible to reliably estimate the pose of the robot
	 */
	public Double distanceToSpeaker() {
		var a = DriverStation.getAlliance();
		if (a.isPresent()) {
			if (a.get() == DriverStation.Alliance.Blue)
				return distanceTo(kBlueSpeakerPosition);
			return distanceTo(kRedSpeakerPosition);
		}
		return null;
	}

	/**
	 * Calculates the distance in meters to the closest speaker.
	 * 
	 * @return the distance to the closest speaker; ; {@code null} if
	 *         it has not been possible to reliably estimate the pose of the robot
	 */
	public Double distanceToClosestSpeaker() {
		Translation2d closestSpeaker = closest(kBlueSpeakerPosition, kRedSpeakerPosition);
		return distanceTo(closestSpeaker);
	}

	/**
	 * Calculates the rotation angle in degrees to the speaker.
	 * 
	 * @return the rotation angle in degrees to the speaker; {@code null} if
	 *         it has not been
	 *         possible to reliably estimate the pose of the robot
	 */
	public Double angleToSpeaker() {
		var a = DriverStation.getAlliance();
		if (a.isPresent()) {
			if (a.get() == DriverStation.Alliance.Blue)
				return angleTo(kBlueSpeakerPosition);
			return angleTo(kRedSpeakerPosition);
		}
		return null;
	}

	/**
	 * Calculates the rotation angle in degrees to the closest speaker.
	 * 
	 * @return the rotation angle in degrees to the closest speaker; {@code null} if
	 *         it has not been possible to reliably estimate the pose of the robot
	 */
	public Double angleToClosestSpeaker() {
		Translation2d closestSpeaker = closest(kBlueSpeakerPosition, kRedSpeakerPosition);
		return angleTo(closestSpeaker);
	}

	/**
	 * Returns the closest to the current position of the robot among the specified
	 * positions.
	 * 
	 * @param positions an array of positions
	 * @return the closest to the current position of the robot among the specified
	 *         positions; {@code null} if it has not been possible to reliably
	 *         estimate the pose of the robot
	 */
	public Translation2d closest(Translation2d... positions) {
		Translation2d closest = null;
		Double min = null;
		for (Translation2d position : positions) {
			Double d = distanceTo(position);
			if (min == null || (min != null && d != null && d < min)) {
				min = d;
				closest = position;
			}
		}
		return closest;
	}

	/**
	 * Returns the closest to the current position of the robot among the specified
	 * positions.
	 * 
	 * @param positions an array of {@code Pose2d}s
	 * @return the closest to the current position of the robot among the specified
	 *         positions; {@code null} if it has not been possible to reliably
	 *         estimate the pose of the robot
	 */
	public Pose2d closest(Pose2d... poses) {
		Pose2d closest = null;
		Double min = null;
		for (Pose2d pose : poses) {
			Double d = distanceTo(pose.getTranslation());
			if (min == null || (min != null && d != null && d < min)) {
				min = d;
				closest = pose;
			}
		}
		return closest;
	}

}
