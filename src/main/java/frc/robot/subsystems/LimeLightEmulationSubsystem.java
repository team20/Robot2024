package frc.robot.subsystems;

import java.io.File;
import java.util.Collection;
import java.util.LinkedList;
import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LimeLightSubsystem.Pose;
import frc.robot.subsystems.PoseEstimationSubsystem.PoseCalculator;
import frc.robot.subsystems.PoseEstimationSubsystemAdvanced.AprilTagMap;

public class LimeLightEmulationSubsystem extends SubsystemBase {

	/**
	 * The path to the "deploy" directory in the project.
	 */
	public final static String s_deployPath = "." + File.separator + "src" + File.separator + "main" + File.separator
			+ "deploy";
	/**
	 * The {@code NetworkTable} named "limelight" which is used by this
	 * {@code LimeLightEmulationSubsystem}.
	 */
	protected NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

	/**
	 * The {@code NetworkTable} named "limelight" which is used by this
	 * {@code LimeLightEmulationSubsystem}.
	 */
	protected NetworkTable table = NetworkTableInstance.getDefault().getTable("AdvantageScope");

	/**
	 * The {@code Pose2d} representing the pose of the robot on the field.
	 */
	private Pose2d m_pose;

	private double m_randomness;

	private LimeLightEmulator m_limelightEmulator;

	private PoseCalculator m_poseCalculator;

	private DriveSubsystem m_driveSubsystem;

	/**
	 * Construcs a {@code LimeLightEmulationSubsystem}.
	 * 
	 * @param pose       a {@code Pose2d} representing the pose of the robot on the
	 *                   field
	 * @param randomness a value (e.g., 0.01) the randomness of the robot modeling
	 *                   the discrepancy between the theory and reality
	 */
	public LimeLightEmulationSubsystem(Pose2d pose, double randomness, DriveSubsystem driveSubsystem) {
		m_pose = pose;
		m_randomness = randomness;
		m_driveSubsystem = driveSubsystem;
		Map<Integer, Pose2d> aprilTagPoses = new TreeMap<Integer, Pose2d>();
		try {
			var m = new AprilTagMap(s_deployPath + File.separator + "2024LimeLightMap.fmap");
			m.forEach((k, v) -> aprilTagPoses.put(k, AprilTagMap.toPose(v)));
		} catch (Exception e) {
			e.printStackTrace();
		}
		m_limelightEmulator = new LimeLightEmulator(() -> {
			return m_pose;
		}, 0.165, 54.0, 4, 0.2, 10.0, 0.1, aprilTagPoses);
		m_poseCalculator = new PoseCalculator() {

			Pose2d previous = null;

			@Override
			public Pose2d pose(Pose2d pose) {
				var current = m_driveSubsystem.getPose();
				if (this.previous == null || pose == null) {
					this.previous = current;
					return pose;
				}
				var t = current.minus(this.previous).times(Math.random() * m_randomness * 2 - m_randomness + 1);
				var refined = pose.plus(t);
				this.previous = current;

				table.getEntry("BotPose@Field")
						.setDoubleArray(AdvantageScopeUtil.toPose2DAdvantageScope(refined));
				return refined;
			}

		};
	}

	/**
	 * A {@code LimeLightEmulator} emulates a LimeLight that detects AprilTags on
	 * the field.
	 * 
	 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
	 * @author Andrew Hwang (u.andrew.h@gmail.com)
	 */
	public static class LimeLightEmulator {

		/**
		 * A {@code Supplier} that provides the {@code Pose2d} of the robot.
		 */
		private Supplier<Pose2d> poseSupplier;

		/**
		 * The width of each AprilTag.
		 */
		private double aprilTagWidth;

		/**
		 * The field of view of the LimeLight.
		 */
		private double limelightFieldOfView;

		/**
		 * The maximum distance for AprilTag detection.
		 */
		private double distanceLimit;

		/**
		 * The bound for artificial positional errors.
		 */
		private double positionalErrorBound;

		/**
		 * The bound for artificial angular errors.
		 */
		private double angularErrorBound;

		/**
		 * The probability that artificial pose outliers occur.
		 */
		private double poseOutlierProbability;

		/**
		 * A {@code Map} that maps the ID of each AprilTag to the {@code Pose2d} of that
		 * AprilTag.
		 */
		protected Map<Integer, Pose2d> aprilTagPoses;

		/**
		 * Constructs a {@code LimeLightEmulator}.
		 * 
		 * @param poseSupplier
		 *                                      a {@code Supplier} that provides the
		 *                                      {@code Pose} of the robot
		 * @param aprilTagWidth
		 *                                      the width of each AprilTag (e.g., 0.165)
		 * @param limelightFieldOfViewInDegrees
		 *                                      the field of view of the LimeLight in
		 *                                      degrees (e.g., 54)
		 * @param distanceLimit
		 *                                      the maximum distance for AprilTag
		 *                                      detection (e.g., 3)
		 * @param positionalErrorBound
		 *                                      the bound for artificial positional
		 *                                      errors (e.g., 0.2)
		 * @param angularErrorBoundInDegrees
		 *                                      the bound for artificial angular errors
		 *                                      in degrees (e.g., 10)
		 * @param poseOutlierProbability
		 *                                      the probability that artificial pose
		 *                                      outliers occur (e.g., 0.1)
		 */
		public LimeLightEmulator(Supplier<Pose2d> poseSupplier, double aprilTagWidth,
				double limelightFieldOfViewInDegrees, double distanceLimit, double positionalErrorBound,
				double angularErrorBoundInDegrees,
				double poseOutlierProbability, Map<Integer, Pose2d> aprilTagPoses) {
			this.poseSupplier = poseSupplier;
			this.aprilTagWidth = aprilTagWidth;
			this.limelightFieldOfView = limelightFieldOfViewInDegrees;
			this.distanceLimit = distanceLimit;
			this.positionalErrorBound = positionalErrorBound;
			this.angularErrorBound = angularErrorBoundInDegrees;
			this.poseOutlierProbability = poseOutlierProbability;
			this.aprilTagPoses = aprilTagPoses;
		}

		/**
		 * Returns the artificial {@code Pose} that this {@code LimeLightEmulator}
		 * currently has.
		 * 
		 * @return the artificial {@code Pose} that this {@code LimeLightEmulator}
		 *         currently has
		 */
		Entry<Pose2d, Collection<Integer>> poseDetected() {
			Pose2d pose = this.poseSupplier.get();
			var tags = visibleAprilTags(pose);
			if (tags.size() == 0)
				return null;
			else {
				pose = new Pose(
						pose.getX() + (2 * positionalErrorBound * Math.random() - positionalErrorBound)
								+ (Math.random() < this.poseOutlierProbability ? 2 : 0),
						pose.getY() + (2 * positionalErrorBound * Math.random() - positionalErrorBound),
						pose.getRotation().getDegrees() + (2 * angularErrorBound * Math.random() - angularErrorBound));
				return Map.entry(pose, tags);
			}
		}

		/**
		 * Returns the IDs of the ApritlTags that should be visible to the robot
		 * according to this
		 * {@code LimeLightEmulator}.
		 * 
		 * @param pose
		 *             the {@code Pose} of the robot
		 * @return the IDs of the ApritlTags that should be visible to the robot
		 *         according to this {@code LimeLightEmulator}
		 */
		Collection<Integer> visibleAprilTags(Pose2d pose) {
			LinkedList<Integer> visibleAprilTags = new LinkedList<Integer>();
			for (Entry<Integer, Pose2d> e : aprilTagPoses.entrySet())
				if (isVisible(e.getValue(), pose))
					visibleAprilTags.add(e.getKey());
			return visibleAprilTags;
		}

		/**
		 * Determines whether or not the AprilTag at the specified {@code Pose} should
		 * be visible to the robot.
		 * 
		 * @param tagPose
		 *                the {@code Pose} of an AprilTag
		 * @param pose
		 *                the {@code Pose} of the robot
		 * @return {@code true} if the AprilTag at the specified {@code Pose} should be
		 *         visible to the robot; {@code false}
		 *         otherwise
		 */

		boolean isVisible(Pose2d tagPose, Pose2d pose) {
			return Math.abs(
					pose.getRotation().minus(tagPose.getRotation()).plus(Rotation2d.fromDegrees(180)).getDegrees()) < 60
					&& tagPose.getTranslation().minus(pose.getTranslation()).getNorm() < distanceLimit
					&& withinViewAngle(new Translation2d(0, -aprilTagWidth / 2).rotateBy(tagPose.getRotation())
							.plus(tagPose.getTranslation()), pose)
					&& withinViewAngle(
							new Translation2d(0, aprilTagWidth / 2).rotateBy(tagPose.getRotation())
									.plus(tagPose.getTranslation()),
							pose);
		}

		/**
		 * Determines whether or not the specified {@code Position} is within the field
		 * of view of the LimeLight.
		 * 
		 * @param position
		 *                 a {@code Position}
		 * @param pose
		 *                 the {@code Pose} of the robot
		 * @return {@code true} if the specified {@code Position} is within the field of
		 *         view of the LimeLight;
		 *         {@code false} otherwise
		 */
		boolean withinViewAngle(Translation2d position, Pose2d pose) {
			return Math.abs(position.minus(pose.getTranslation()).getAngle().minus(pose.getRotation())
					.getDegrees()) < limelightFieldOfView / 2;
		}

	};

	@Override
	public void periodic() {
		var e = m_limelightEmulator.poseDetected();
		if (e != null) {
			var p = toDoubleArray(e.getKey());
			limelightTable.getEntry("botpose").setDoubleArray(p);
			limelightTable.getEntry("json").setString(json(e.getKey(), e.getValue()));
			limelightTable.getEntry("targetpose_robotspace").setDoubleArray(poseArray(e.getKey(), e.getValue()));
		} else
			limelightTable.getEntry("botpose")
					.setDoubleArray(new double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Math.random() });
		m_pose = m_poseCalculator.pose(m_pose); // introduces a little delay
	}

	private double[] poseArray(Pose2d pose, Collection<Integer> tagIDs) {
		var poses = new LinkedList<Pose>();
		for (Integer tagID : tagIDs) {
			var tagPose = this.m_limelightEmulator.aprilTagPoses.get(tagID);
			Transform2d t = tagPose.minus(pose);
			var tt = t.getTranslation().rotateBy(Rotation2d.fromDegrees(90));
			poses.add(new Pose(tt.getX(), tt.getY(), tt.getAngle().getDegrees()));
		}
		pose = Pose.average(poses.toArray(new Pose[0]));
		return new double[] { pose.getX(), 0.0, pose.getY(), 0.0, 0.0, pose.getRotation().getDegrees() };
	}

	private String json(Pose2d pose, Collection<Integer> tagIDs) {
		String s = "";
		for (Integer tagID : tagIDs) {
			if (s.length() > 0)
				s += ", ";
			var tagPose = this.m_limelightEmulator.aprilTagPoses.get(tagID);
			Transform2d t = tagPose.minus(pose);
			var tt = t.getTranslation().rotateBy(Rotation2d.fromDegrees(90));
			s += String.format("{ \"fID\": %s, \"t6t_rs\": [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]}", "" + tagID,
					tt.getX(), 0.0,
					tt.getY(),
					0.0, 0.0,
					0.0);
		}
		return "{ \"Results\": {\"Fiducial\": [" + s + "]}}";
	}

	private double[] toDoubleArray(Pose2d poseDetected) {
		if (poseDetected == null)
			return new double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		return new double[] { poseDetected.getX(), poseDetected.getY(), 0.0, 0.0, 0.0,
				poseDetected.getRotation().getDegrees() };
	}
}
