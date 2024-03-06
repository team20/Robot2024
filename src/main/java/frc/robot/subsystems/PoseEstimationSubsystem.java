package frc.robot.subsystems;

import java.util.Collection;
import java.util.Map;
import java.util.TreeMap;
import java.util.function.Supplier;
import java.util.stream.Stream;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The purpose of the {@code PoseEstimationSubsystem} is to provide the pose of
 * the robot, each AprilTag, as well as others of interest. For stationary
 * objects such AprilTags, it stores the corresponding {@code Pose2d}s and
 * provide
 * them as needed.
 * For a moving object such as the robot, it estimates the pose based on a
 * variety of sources including LimeLight as well as encoders and sensors
 * attached to the robot.
 * 
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 */
public class PoseEstimationSubsystem extends LimeLightSubsystem {

	/**
	 * A {@code PoseCalculator} calculates the pose of an object based on the pose
	 * of that object at an earlier time and some
	 * changes in that object observed via some sources such as a gyroscope,
	 * encoders, etc.
	 * 
	 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
	 * @author Andrew Hwang (u.andrew.h@gmail.com)
	 */
	public interface PoseCalculator {

		/**
		 * Calculates the current pose of an object based on the pose of that object at
		 * an earlier time and some changes in
		 * that object observed via some sources such as a gyroscope, encoders, etc.
		 * 
		 * @param pose
		 *             a {@code Pose} representing the pose of the object at an earlier
		 *             time
		 * @return a {@code Pose} representing the pose calculated by this
		 *         {@code PoseCalculator}
		 */
		public Pose2d pose(Pose2d pose);

	}

	/**
	 * A {@code PoseEstimator} estimates the pose of an object based on sample
	 * {@code Pose2d}s.
	 * 
	 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
	 * @author Andrew Hwang (u.andrew.h@gmail.com)
	 */
	public class PoseEstimator {

		/**
		 * The distance threshold for outlier detection (a sample {@code Pose2d} is
		 * considered an outlier and rejected if its
		 * distance from the estimated {@code Pose2d} is larger than this threshold).
		 */
		protected double distanceThreshold;

		/**
		 * The {@code Pose2d} representing the pose estimated by this
		 * {@code PoseEstimator}.
		 */
		protected Pose2d estimatedPose = null;

		/**
		 * The number of {@code Pose2d}s that have been considered outliers by this
		 * {@code PoseEstimator}.
		 */
		protected int outliers = 0;

		/**
		 * The start time (in milliseconds) of this {@code PoseEstimator}.
		 */
		protected long startTime = System.currentTimeMillis();

		/**
		 * The number of consecutive rejections of sample {@code Pose2d}s needed to
		 * reset this {@code PoseEstimator}.
		 */
		int rejectionLimit;

		/**
		 * The number of consecutive sample {@code Pose2d}s that have been rejected as
		 * outliers.
		 */
		int rejections = 0;

		/**
		 * The weight applied to each sample {@code Pose2d}.
		 */
		protected double weight;

		/**
		 * Constructs a {@code PoseEstimator}.
		 * 
		 * @param distanceThreshold
		 *                          the distance threshold for outlier detection (a
		 *                          sample {@code Pose2d} is considered an outlier and
		 *                          rejected if its distance from the estimated
		 *                          {@code Pose2d} is larger than this threshold)
		 * @param rejectionLimit
		 *                          the number of rejections needed to reset the
		 *                          {@code PoseEstimatorWeighted}
		 * @param weight
		 *                          the weight of each sample {@code Pose2d}
		 */
		public PoseEstimator(double distanceThreshold, int rejectionLimit, double weight) {
			this.distanceThreshold = distanceThreshold;
			this.rejectionLimit = rejectionLimit;
			this.weight = weight;
		}

		/**
		 * Returns a {@code Pose2d} representing the pose estimated by this
		 * {@code PoseEstimator}.
		 * 
		 * @return a {@code Pose2d} representing the pose estimated by this
		 *         {@code PoseEstimator}
		 */
		public Pose2d estimatedPose() {
			return estimatedPose;
		}

		/**
		 * Updates this {@code PoseEstimator} based on the specified sample
		 * {@code Pose2d}.
		 * 
		 * @param sample
		 *               a sample {@code Pose2d}
		 * @return {@code false} if the specified sample {@code Pose2d} is considered an
		 *         outlier (because the x- or
		 *         y-coordinate value of the sample {@code Pose2d} is different by more
		 *         than the threshold compared to the
		 *         estimated {@code Pose2d}) and thus rejected; {@code true} if this
		 *         {@code PoseEstimator} is updated based on
		 *         the specified {@code Pose2d}
		 */
		public final boolean update(Pose2d sample) {
			if (isOutlier(sample))
				return false;
			estimatedPose(sample);
			return true;
		}

		/**
		 * Returns the number of outliers (i.e., sample {@code Pose2d}s that have been
		 * rejected by this
		 * {@code PoseEstimator}).
		 * 
		 * @return the number of outliers (i.e., sample {@code Pose2d}s that have been
		 *         rejected by this {@code PoseEstimator})
		 */
		public int outliers() {
			return outliers;
		}

		/**
		 * Determines whether or not the specified sample {@code Pose2d} is an outlier.
		 * 
		 * @param sample
		 *               a sample {@code Pose2d}
		 * @return {@code true} if either the x- or y-coordinate value of the sample
		 *         {@code Pose2d} is different by more than
		 *         the threshold compared to the estimated {@code Pose2d} maintained by
		 *         this {@code PoseEstimator};
		 *         {@code false} otherwise
		 */
		protected boolean isOutlier(Pose2d sample) {
			if (sample == null || this.estimatedPose == null)
				return false;
			if (hasNaN(sample))
				return true;
			Pose2d error = error(sample, this.estimatedPose);
			if (Math.abs(error.getX()) > distanceThreshold || Math.abs(error.getY()) > distanceThreshold) {
				outliers++;
				if (++rejections > rejectionLimit)
					reset();
				return true;
			} else {
				if (sample != null && this.estimatedPose != null)
					rejections = 0;
				return false;
			}
		}

		/**
		 * /** Resets this {@code PoseEstimator}.
		 */
		protected void reset() {
			System.out.println("resetting the pose estimator...");
			estimatedPose = null;
			rejections = 0;
		}

		/**
		 * Updates the estimated {@code Pose2d} based on the specified sample
		 * {@code Pose2d}.
		 * 
		 * @param sample
		 *               a sample {@code Pose2d}
		 */
		public void estimatedPose(Pose2d sample) {
			if (sample != null) {
				this.estimatedPose = weightedSum(sample, weight, this.estimatedPose, 1 - weight);
				if (hasNaN(this.estimatedPose))
					reset();
			}
		}

		/**
		 * Calculates the discrepancies between the specified {@code Pose2d}s.
		 * 
		 * @param pose
		 *                  a {@code Pose2d}
		 * @param reference
		 *                  a reference {@code Pose2d} for comparison
		 * @return a {@code Pose2d} representing the difference between the
		 *         {@code Pose2d}s
		 *         ({@code null} if either of the given
		 *         {@code Pose2d}s is {@code null})
		 */
		public static Pose2d error(Pose2d pose, Pose2d reference) {
			if (pose == null || reference == null)
				return null;
			return new Pose2d(pose.getX() - reference.getX(), pose.getY() - reference.getY(),
					pose.getRotation().minus(reference.getRotation()));
		}

		/**
		 * Calculates the specified weighted sum.
		 * 
		 * @param p1
		 *           the first {@code Pose2d}
		 * @param w1
		 *           the weight of the first {@code Pose2d}
		 * @param p2
		 *           the second {@code Pose2d}
		 * @param w2
		 *           the weight of the second {@code Pose2d}
		 * @return the weighted sum of the specified {@code Pose2d}s
		 */
		public static Pose2d weightedSum(Pose2d p1, double w1, Pose2d p2, double w2) {
			if (p1 == null)
				return p2;
			if (p2 == null)
				return p1;
			double a1 = p1.getRotation().getRadians();
			double a2 = p2.getRotation().getRadians();
			if (a1 > a2 + Math.PI)
				a2 += 2 * Math.PI;
			else if (a2 > a1 + Math.PI)
				a1 += 2 * Math.PI;
			return new Pose2d(p1.getX() * w1 + p2.getX() * w2, p1.getY() * w1 + p2.getY() * w2,
					Rotation2d.fromRadians(a1 * w1 + a2 * w2));
		}

		/**
		 * Determines whether or not any coordinate or yaw value of this {@code Pose2d}
		 * is NaN.
		 * 
		 * @return {@code true} if any coordinate or yaw value of this {@code Pose2d} is
		 *         NaN; {@code false} otherwise
		 */
		public static boolean hasNaN(Pose2d pose) {
			return Double.isNaN(pose.getX()) || Double.isNaN(pose.getY())
					|| Double.isNaN(pose.getRotation().getRadians());
		}

		/**
		 * Updates the {@code Pose} estimated by this {@code PoseEstimator} based on the
		 * specified {@code PoseCalculator}s.
		 * 
		 * @param poseCalculators
		 *                        {@code PoseCalculator}s
		 */
		public void update(Collection<PoseCalculator> poseCalculators) {
			if (poseCalculators.size() > 0) {
				Stream<Pose2d> poses = poseCalculators.stream().map(c -> c.pose(estimatedPose));
				estimatedPose = Pose.average(poses.toList().toArray(new Pose2d[0]));
			}
		}

	}

	/**
	 * The {@code PoseEstimator} for estimating the pose of the robot.
	 */
	protected PoseEstimator m_poseEstimator = new PoseEstimator(1.0, 10, 0.1);

	/**
	 * The {@code PoseCalculator}s for enhancing the accuracy of the estimated pose
	 * based on data from various sources
	 * such as a gyroscope, encoders, etc.
	 */
	protected Map<String, PoseCalculator> m_poseCalculators = new TreeMap<String, PoseCalculator>();

	/**
	 * Constructs a {@code PoseEstimationSubsystem}.
	 */
	public PoseEstimationSubsystem() {
		super();
	}

	/**
	 * Returns a {@code Pose2d} representing the estimated pose of the robot.
	 * 
	 * @return a {@code Pose2d} representing the estimated pose of the robot
	 *         ({@code null} if it has not been possible to reliably estimate the
	 *         pose of the robot)
	 */
	public Pose2d estimatedPose() {
		var pose = m_poseEstimator.estimatedPose();
		return pose == null || PoseEstimator.hasNaN(pose) || pose.equals(DEFAULT_POSE) ? null : pose;
	}

	/**
	 * Adds a {@code Supplier<Pose>} which can provide {@code Pose}s obtained from
	 * some sources such as a gyroscope,
	 * encoders, etc. in order to enhance the accuracy of the pose estimated by this
	 * {@code PoseEstimator}.
	 * 
	 * @param label
	 *                     a label associated with the specifiled
	 *                     {@code Supplier<Pose>}
	 * @param poseSupplier
	 *                     a {@code Supplier<Pose>} which can provide {@code Pose}s
	 *                     obtained from some sources such as a
	 *                     gyroscope, encoders, etc.
	 */
	public void addPoseSupplier(String label, Supplier<Pose2d> poseSupplier) {
		this.m_poseCalculators.put(label, new PoseCalculator() {

			Pose2d previous = null;

			@Override
			public Pose2d pose(Pose2d pose) {
				var current = poseSupplier.get();
				if (this.previous == null || pose == null) {
					this.previous = current;
					return pose;
				}
				var refined = pose.plus(current.minus(this.previous));
				this.previous = current;
				return refined;
			}

		});
	}

	/**
	 * Is invoked whenever the "botpose" entry in the "limelight" table changes.
	 * 
	 * @param event a {@code NetworkTableEvent} regarding the change in the
	 *              "botpose" entry in the "limelight" table
	 * @return a {@code TimestampedDoubleArray} representing the pose of the robot
	 *         in terms of the x and y-coordinate values and the yaw value (the
	 *         orientation relative to the positive x-axis) in
	 *         degrees
	 */
	@Override
	protected double[] changedBotPose(NetworkTableEvent event) {
		boolean validSample = false;
		try {
			var v = event.valueData.value;
			m_botpose = v.getDoubleArray();
			if (m_botpose != null) {
				var pose = new Pose2d(m_botpose[0], m_botpose[1], Rotation2d.fromDegrees(m_botpose[5]));
				if (Math.abs(pose.getX()) > 0.1 || Math.abs(pose.getY()) > 0.1) // if botpose seems reasonable
					if (m_poseEstimator.update(pose))
						validSample = true;
			}
			return m_botpose;
		} catch (Exception e) {
			e.printStackTrace();
			return null;
		} finally {
			m_confidence = 0.96 * m_confidence + (validSample ? 0.04 : 0);
		}
	}

	/**
	 * Is called periodically by the {@link CommandScheduler}.
	 */
	@Override
	public void periodic() {
		super.periodic();
		m_poseEstimator.update(m_poseCalculators.values());
		try {
			SmartDashboard.putNumber("pose estimation: confidence",
					confidence());
			var pose = estimatedPose();
			if (pose != null) // pose data that can be used by AdvantageScope
				SmartDashboard.putNumberArray("pose estimation: pose estimated",
						new double[] { pose.getX() + 8.27, pose.getY() + 4.05, pose.getRotation().getRadians() });
			SmartDashboard.putNumber("pose estimation: rotation angle to the closest speaker (degrees)",
					angleToClosestSpeaker());
			SmartDashboard.putNumber("pose estimation: distance to the closest speaker (meters)",
					distanceToClosestSpeaker());
			if (DriverStation.getAlliance().isPresent()) {
				Alliance alliance = DriverStation.getAlliance().get();
				SmartDashboard.putNumber("pose estimation: rotation angle to " + alliance + " speaker (degrees)",
						angleToSpeaker());
				SmartDashboard.putNumber("pose estimation: distance to " + alliance + " speaker (meters)",
						distanceToSpeaker());
			}
		} catch (Exception e) {
			// e.printStackTrace();
		}
	}

}
