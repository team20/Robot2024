package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;

public class AdvantageScopeUtil {

	/**
	 * Converts the specified {@code Pose2d} into the {@code double} array
	 * representation that AdvatageScope can consume.
	 * 
	 * @param pose a {@code Pose2d}
	 * @return a {@code double} array representation of the specified {@code Pose2d}
	 */
	public static double[] toPose2DAdvantageScope(Pose2d pose) {
		return pose == null ? new double[0]
				: toPose2DAdvantageScope(pose.getX(), pose.getY(),
						pose.getRotation().getDegrees());
	}

	/**
	 * Converts the specified pose data into the {@code double} array
	 * representation that AdvatageScope can consum.
	 * 
	 * @param x            the x-coordinate value
	 * @param y            the y-coordinate value
	 * @param yawInDegrees the yaw in degrees
	 * @return a {@code double} array representation of the specified pose data
	 */
	public static double[] toPose2DAdvantageScope(double x, double y, double yawInDegrees) {
		return new double[] { x + 8.27, y + 4.05, yawInDegrees * Math.PI / 180 };
	}

}
