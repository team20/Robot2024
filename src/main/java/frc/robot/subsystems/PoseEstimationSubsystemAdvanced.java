package frc.robot.subsystems;

import java.io.File;
import java.util.Arrays;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.TreeMap;
import java.util.function.Consumer;
import java.util.stream.Stream;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedObject;

/**
 * The purpose of the {@code PoseEstimationSubsystem} is to provide the pose of
 * the robot, each AprilTag, as well as others of interest. For stationary
 * objects such AprilTags, it stores the corresponding {@code Pose}s and provide
 * them as needed.
 * For a moving object such as the robot, it estimates the pose based on a
 * variety of sources including LimeLight as well as encoders and sensors
 * attached to the robot.
 * 
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 */
public class PoseEstimationSubsystemAdvanced extends PoseEstimationSubsystem {

	/**
	 * A {@code AprilTagMap} maps the ID of each AprilTag to the 3D transform
	 * representing the pose of that AprilTag.
	 * 
	 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
	 * @author Andrew Hwang (u.andrew.h@gmail.com)
	 */
	public static class AprilTagMap extends HashMap<Integer, double[]> {

		/**
		 * The automatically generated serial version UID.
		 */
		private static final long serialVersionUID = -7392062114679722757L;

		/**
		 * Constructs an {@code AprilTagMap} by parsing the specified JSON file.
		 * 
		 * @param fileName
		 *                 the name of the file
		 */
		public AprilTagMap(String fileName) {
			try {
				JsonNode root = new ObjectMapper().readTree(new File(fileName));
				JsonNode poses = root.path("fiducials");
				Iterator<JsonNode> i = poses.elements();
				while (i.hasNext()) {
					JsonNode n = i.next();
					int tagID = n.path("id").asInt();
					Iterator<JsonNode> j = n.path("transform").elements();
					double[] transform = new double[] { j.next().asDouble(), j.next().asDouble(), j.next().asDouble(),
							j.next().asDouble(), j.next().asDouble(), j.next().asDouble(), j.next().asDouble(),
							j.next().asDouble(), j.next().asDouble(), j.next().asDouble(), j.next().asDouble(),
							j.next().asDouble() };
					put(tagID, transform);
				}
			} catch (Exception e1) {
				e1.printStackTrace();
			}
			System.out.println(size() + " tags read from \"" + fileName + "\".");
		}

		/**
		 * Constructs a {@code Pose2d} from the specified 3D transformation.
		 * 
		 * @param transform
		 *                  a {@code double} array representing a 3D transformation
		 * @return a {@code Pose2d} constructed from the specified 3D transformation
		 */
		public static Pose2d toPose(double[] transform) {
			double mxx = transform[0];
			double mxy = transform[1];
			double tx = transform[3];
			double ty = transform[7];
			return new Pose2d(tx, ty, Rotation2d.fromRadians(-Math.atan2(mxy, mxx)));
		}

	}

	/**
	 * The path to the "deploy" directory in the project.
	 */
	public final static String s_deployPath = "." + File.separator + "src" + File.separator + "main" + File.separator
			+ "deploy";
	/**
	 * The {@code NetworkTable} named "vision" which is used by this
	 * {@code AprilTagSubsystem}.
	 */
	protected NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("AdvantageScope");

	/**
	 * A {@code Map} that maps the ID of each AprilTag to the {@code Pose2d} of that
	 * AprilTag.
	 */
	protected Map<Integer, Pose2d> m_aprilTagPoses = new TreeMap<Integer, Pose2d>();

	/**
	 * The most recent data about recognized AprilTags.
	 */
	protected TimestampedObject<Map<String, double[]>> m_tags;

	/**
	 * Returns a {@code Pose2d} representing the estimated pose of the specified
	 * AprilTag from the perspective of the robot.
	 *
	 * @param tagID the ID of an AprilTag
	 * @return a {@code Pose2d} representing the estimated pose of the specified
	 *         AprilTag from the perspective of the robot
	 */
	public Pose2d getPose(String tagID) {
		if (m_tags == null)
			return null;
		var v = m_tags.value.get(tagID);
		return new Pose(v[0], v[1], v[2]);
	}

	/**
	 * Returns the most recent data about recognized AprilTags.
	 *
	 * @return a {@code Map} that maps the ID of each recognized AprilTag to a
	 *         {@code double} array representing the displacement from the camera to
	 *         that AprilTag. Each {@code double} array contains the x and
	 *         y-coordinate values and the orientation in degrees of the
	 *         displacement.
	 */
	public TimestampedObject<Map<String, double[]>> tags() {
		return m_tags;
	}

	/**
	 * Returns the distance to the specified target position
	 *
	 * @param targetPosition the target position
	 * @return the distance to the specified target position
	 */
	public double getDistance(Translation2d targetPosition) {
		return getDistance(estimatedPose(), targetPosition);
	}

	/**
	 * Returns the distance to the specified target AprilTag
	 *
	 * @param targetPosition the ID of the AprilTag
	 * @return the distance to the specified target AprilTag
	 */
	public Double getDistance(String tagID) {
		var pose = getPose(tagID);
		return pose == null ? null : pose.getTranslation().getNorm();
	}

	/**
	 * Returns the {@code Rotation2d} that is needed to turn the robot toward the
	 * specified target position
	 *
	 * @param targetPosition the target position
	 * @return the {@code Rotation2d} that is needed to turn the robot toward the
	 *         specified target position
	 */
	public Rotation2d getRotation(Translation2d targetPosition) {
		return estimatedPose() == null ? null
				: getRotation(estimatedPose(),
						targetPosition);
	}

	/**
	 * Returns the {@code Rotation2d} that is needed to turn the robot toward the
	 * specified AprilTag
	 *
	 * @param targetPosition the ID of the AprilTag
	 * @return the {@code Rotation2d} that is needed to turn the robot toward the
	 *         specified AprilTag
	 */
	public Rotation2d getRotation(String tagID) {
		var pose = getPose(tagID);
		return pose == null ? null
				: pose.getTranslation().getAngle()
						.minus(Rotation2d.fromDegrees(90));
	}

	/**
	 * Returns the {@code Rotation2d} that is needed to turn the robot toward the
	 * midpoint of the detected AprilTags.
	 *
	 * @param targetPosition the ID of the AprilTag
	 * @return the {@code Rotation2d} that is needed to turn the robot toward the
	 *         midpoint of the detected AprilTags
	 */
	public Rotation2d getRotationToDetectedTags() {
		if (m_tags == null)
			return null;
		var values = m_tags.value.values();
		if (values == null || values.size() == 0)
			return null;
		var l = values.stream().map(v -> new Pose2d(new Translation2d(v[0], v[1]),
				Rotation2d.fromDegrees(v[2])))
				.toList();
		Pose2d pose = Pose.average(l.toArray(new Pose2d[0]));
		return pose == null ? null
				: pose.getTranslation().getAngle()
						.minus(Rotation2d.fromDegrees(90));
	}

	/**
	 * Calculates the target {@code Pose2d} based on the current pose of the robot
	 * and the specified target position and distance to the target.
	 *
	 * @param targetPosition   the target position whose x and y-coordinate values
	 *                         are in meters
	 * @param distanceToTarget the desired distance to the target
	 * @return the target {@code Pose2d} calculated based on the current pose of
	 *         the
	 *         robot and the specified target position and distance to the target
	 * @throw UnsupportedOperationException if the current robot pose and
	 *        the target are at the same location
	 */
	public Pose2d getTargetPose(Translation2d target, double distanceToTarget) {
		return estimatedPose() == null ? null
				: getTargetPose(estimatedPose(),
						target, distanceToTarget);
	}

	/**
	 * Returns the distance from the specified {@code Pose2d} to the specified
	 * target position.
	 *
	 * @param pose   a {@code Pose2d}
	 * @param target a target position
	 * @return the distance from the specified {@code Pose2d} to the specified
	 *         target position
	 */
	public static double getDistance(Pose2d pose, Translation2d target) {
		return pose.getTranslation().getDistance(target);
	}

	/**
	 * Returns the {@code Rotation2d} that is needed to turn the robot toward the
	 * specified target position
	 *
	 * @param pose           a {@code Pose2d} representing the pose of the robot
	 * @param targetPosition the target position
	 * @return the {@code Rotation2d} that is needed to turn the robot toward the
	 *         specified target position
	 */
	public static Rotation2d getRotation(Pose2d pose, Translation2d targetPosition) {
		return targetPosition.minus(pose.getTranslation()).getAngle().minus(pose.getRotation());
	}

	/**
	 * Returns a {@code Transform2d} for mapping the specified {@code Pose2d} to
	 * the
	 * specified target position.
	 *
	 * @param pose           a {@code Pose2d}
	 * @param targetPosition the target position
	 * @return a {@code Transform2d} for mapping the specified {@code Pose2d} to
	 *         the
	 *         specified target position
	 */
	public static Transform2d getTransform(Pose2d pose, Translation2d targetPosition) {
		return new Transform2d(pose, getTargetPose(pose, targetPosition, 0));
	}

	/**
	 * Calculates the target {@code Pose2d} based on the specified current
	 * {@code Pose2d}, target position, and distance to the target.
	 *
	 * @param currentPose      the current {@code Pose2d} of the robot
	 * @param targetPosition   the target position whose x and y-coordinate values
	 *                         are in meters
	 * @param distanceToTarget the desired distance to the target
	 * @return the target {@code Pose2d} calculated based on the specified current
	 *         {@code Pose2d}, target position, and distance to the target
	 * @throw UnsupportedOperationException if the specified {@code Pose2d} and
	 *        target are at the same location
	 */
	public static Pose2d getTargetPose(Pose2d pose, Translation2d target, double distanceToTarget) {
		Translation2d diff = target.minus(pose.getTranslation());
		if (diff.getNorm() == 0)
			throw new UnsupportedOperationException();
		return new Pose2d(pose.getTranslation().plus(diff.times(1 - distanceToTarget
				/ diff.getNorm())),
				diff.getAngle());
	}

	/**
	 * Parses the specified JSON expression.
	 *
	 * @param s a {@code String} containing a JSON expression
	 * @return a {@code Map} that maps the ID of each recognized AprilTag to a
	 *         {@code double} array representing the displacement from the camera to
	 *         that AprilTag. Each {@code double} array contains the x and
	 *         y-coordinate values and the yaw value (i.e., the orientation relative
	 *         to the positive x-axis) in degrees of the displacement
	 * @throws JsonMappingException    if a parsing error occurs
	 * @throws JsonProcessingException if a parsing error occurs
	 */
	protected static Map<String, double[]> toMap(String s) throws JsonMappingException, JsonProcessingException {
		JsonNode n = new ObjectMapper().readTree(s);
		n = n.path("Results").path("Fiducial");
		var m = new TreeMap<String, double[]>();
		n.forEach(e -> {
			var i = e.path("t6t_rs").elements();
			double v1 = i.next().asDouble();
			i.next();
			double v2 = i.next().asDouble();
			i.next();
			i.next();
			double v3 = i.next().asDouble();
			var v = new double[] { v1, v2, v3 };
			m.put(e.path("fID").toString(), v);
		});
		return m;
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
	public static void subscribe(String tableName, String topicName, String defaultValue,
			Consumer<NetworkTableEvent> listener) {
		NetworkTableInstance i = NetworkTableInstance.getDefault();
		NetworkTable t = i.getTable(tableName);
		var s = t.getStringTopic(topicName).subscribe(defaultValue);
		i.addListener(s, EnumSet.of(NetworkTableEvent.Kind.kValueAll), listener);
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
	 * Constructs a {@code PoseEstimationSubsystem}.
	 */
	public PoseEstimationSubsystemAdvanced() {
		try {
			subscribe("limelight", "json", "", event -> changedJson(event));
			var m = new AprilTagMap(s_deployPath + File.separator + "2024LimeLightMap.fmap");
			m.forEach((k, v) -> m_aprilTagPoses.put(k, AprilTagMap.toPose(v)));
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	/**
	 * Returns the {@code Pose} of the specified AprilTag.
	 * 
	 * @param tagID the ID of the AprilTag
	 * @return the {@code Pose} of the specified AprilTag ({@code null} if there is
	 *         no such AprilTag)
	 */
	public Pose2d aprilTagPose(int tagID) {
		return m_aprilTagPoses.get(tagID);
	}

	/**
	 * Returns, for each detected AprilTag, the distance in meters to that AprilTag.
	 * 
	 * @return a {@code Map} containing, for each detected AprilTag, the distance in
	 *         meters to that AprilTag
	 */
	public Map<Integer, Double> getDistancesToDetectedTags() {
		var pose = super.estimatedPose();
		return pose == null ? Map.of() : getDistancesToDetectedTags(pose);
	}

	/**
	 * Returns, for each detected AprilTag, the rotation needed for the specified
	 * {@code Pose2d} to face toward that AprilTag.
	 * 
	 * @return a {@code Map} containing, for each detected AprilTag, the rotation
	 *         needed for the specified {@code Pose2d} face toward that AprilTag
	 */
	public Map<Integer, Rotation2d> getRotationsToDetectedTags(Pose2d pose) {
		var m = new TreeMap<Integer, Rotation2d>();
		if (m_tags == null)
			return m;
		for (String s : m_tags.value.keySet()) {
			try {
				int i = Integer.parseInt(s);
				var p = m_aprilTagPoses.get(i);
				m.put(i, getRotation(pose, p.getTranslation()));
			} catch (Exception e) {
			}
		}
		return m;
	}

	/**
	 * Returns, for each detected AprilTag, the rotation needed for the robot to
	 * face toward that AprilTag.
	 * 
	 * @return a {@code Map} containing, for each detected AprilTag, the rotation
	 *         needed for the robot face toward that AprilTag
	 */
	public Map<Integer, Rotation2d> getRotationsToDetectedTags() {
		var pose = super.estimatedPose();
		return pose == null ? Map.of() : getRotationsToDetectedTags(pose);
	}

	/**
	 * Is invoked periodically (every 20 ms approximately).
	 */
	@Override
	public void periodic() {
		super.periodic();
		var pose = estimatedPose();
		visionTable.getEntry("Pose Estimated").setString("" + pose);
		if (pose != null)
			visionTable.getEntry("BotPose'").setDoubleArray(AdvantageScopeUtil.toPose2DAdvantageScope(pose.getX(),
					pose.getY(), pose.getRotation().getDegrees()));
	}

	/**
	 * Is invoked whenever there is a change in the {@code NetworkTable} entry
	 * representing the pose of the robot.
	 */
	protected double[] changedBotPose(NetworkTableEvent event) {
		var botpose = super.changedBotPose(event);
		if (botpose != null) {
			visionTable.getEntry("BotPose")
					.setDoubleArray(
							AdvantageScopeUtil.toPose2DAdvantageScope(m_botpose[0], m_botpose[1],
									m_botpose[5]));
		}
		return botpose;
	}

	/**
	 * Is invoked whenever the "json" entry in the "limelight" table changes.
	 *
	 * @param event a {@code NetworkTableEvent} regarding the change in the
	 *              "json" entry in the "limelight" table
	 * @return a {@code TimestampedObject} containing a {@code Map} that maps the
	 *         ID
	 *         of each recognized AprilTag to a
	 *         {@code double} array representing the displacement from the camera to
	 *         that AprilTag. Each {@code double} array contains the x and
	 *         y-coordinate values and the yaw value (i.e., the orientation relative
	 *         to the positive x-axis) in degrees of the displacement
	 */
	protected TimestampedObject<Map<String, double[]>> changedJson(NetworkTableEvent event) {
		try {
			var v = event.valueData.value;
			var m = toMap(v.getString());
			m_tags = new TimestampedObject<Map<String, double[]>>(v.getTime(),
					v.getServerTime(), m);
		} catch (Exception e) {
			m_tags = null;
			e.printStackTrace();
		}
		if (m_tags != null)
			visionTable.getEntry("Json").setString(toString(m_tags.value));
		return m_tags;
	}

	/**
	 * Returns, for each detected AprilTag, the distance in meters to that AprilTag
	 * from the specified {@code Pose2d}
	 * 
	 * @param pose a {@code Pose2d}
	 * @return a {@code Map} containing, for each detected AprilTag, the distance in
	 *         meters to that AprilTag from the specified {@code Pose2d}
	 */
	public Map<Integer, Double> getDistancesToDetectedTags(Pose2d pose) {
		var m = new TreeMap<Integer, Double>();
		if (m_tags == null)
			return m;
		for (String s : m_tags.value.keySet()) {
			try {
				int i = Integer.parseInt(s);
				var p = m_aprilTagPoses.get(i);
				m.put(i, getDistance(pose, p.getTranslation()));
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		return m;
	}

	/**
	 * Returns a string representation of the specified {@code Map}.
	 * 
	 * @param m a {@code Map} that maps the ID of each recognized AprilTag to a
	 *          {@code double} array representing the displacement from the camera
	 *          to that AprilTag. Each {@code double} array contains the x and
	 *          y-coordinate values and the yaw value (i.e., the orientation
	 *          relative to the positive x-axis) in degrees of the displacement
	 * @return a string representation of the specified {@code Map}
	 */
	String toString(Map<String, double[]> m) {
		if (m != null && m.size() > 0) {
			Stream<String> stream = m.entrySet().stream().map(e -> e.getKey() + "=" + Arrays.toString(e.getValue()));
			return stream.reduce((e1, e2) -> e1 + "," + e2).get();
		}
		return "";
	}

	/**
	 * Records the specified value in the specified entry in a {@code NetworkTable}.
	 * 
	 * @param entryName the name of the entry
	 * @param value     the value to record
	 */
	public void recordPose(String entryName, Pose2d value) {
		if (value != null && !(value instanceof Pose))
			value = new Pose(value.getX(), value.getY(), value.getRotation().getDegrees());
		if (value == null)
			visionTable.getEntry(entryName).setDoubleArray(new double[0]);
		else
			visionTable.getEntry(entryName).setDoubleArray(AdvantageScopeUtil.toPose2DAdvantageScope(value.getX(),
					value.getY(), value.getRotation().getDegrees()));
	}

	/**
	 * Records the specified value in the specified entry in a {@code NetworkTable}.
	 * 
	 * @param entryName the name of the entry
	 * @param value     the value to record
	 */
	public void recordString(String entryName, String value) {
		visionTable.getEntry(entryName).setString(value);
	}

}
