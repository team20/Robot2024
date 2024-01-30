package hlib.drive;

/**
 * A {@code Pose} represents a pose, defined as a position with an orientation (direction) in a 2-dimensional space.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public class Pose extends Position {

	/**
	 * The default {@code Pose} located at the origin of the space.
	 */
	public static final Pose DEFAULT_POSE = new Pose(0, 0, 0);

	/**
	 * The orientation of this {@code Pose}, expressed as a yaw value in radians.
	 */
	protected double yaw;

	/**
	 * Constructs a {@code Pose}.
	 * 
	 * @param x
	 *            the x-coordinate value of the {@code Pose}
	 * @param y
	 *            the y-coordinate value of the {@code Pose}
	 * @param yawInRadians
	 *            the orientation of the {@code Pose} in radians
	 */
	public Pose(double x, double y, double yawInRadians) {
		super(x, y);
		this.yaw = normalize(yawInRadians);
	}

	/**
	 * Constructs a {@code Pose}.
	 * 
	 * @param p
	 *            the {@code Position} of the {@code Pose}
	 * @param yawInRadians
	 *            the orientation of the {@code Pose} in radians
	 */
	public Pose(Position p, double yawInRadians) {
		this(p.x, p.y, yawInRadians);
	}

	/**
	 * Returns the orientation of this {@code Pose}, expressed as a yaw value in radians.
	 * 
	 * @return the orientation of this {@code Pose}, expressed as a yaw value in radians
	 */
	public double yawInRadians() {
		return yaw;
	}

	/**
	 * Returns the orientation of this {@code Pose}, expressed as a yaw value in degrees.
	 * 
	 * @return the orientation of this {@code Pose}, expressed as a yaw value in degrees
	 */
	public double yawInDegrees() {
		return Math.toDegrees(yaw);
	}

	/**
	 * Returns a {@code String} representation of this {@code Pose}.
	 * 
	 * @return a {@code String} representation of this {@code Pose}
	 */
	@Override
	public String toString() {
		return String.format("(%.3f, %.3f, %.1f degrees)", x, y, Math.toDegrees(yaw));
	}

	/**
	 * Determines whether or not the given {@code Object} is equal to this {@code Pose}.
	 * 
	 * @return {@code true} if the given {@code Object} is equal to this {@code Pose}; {@code false} otherwise
	 */
	@Override
	public boolean equals(Object o) {
		if (o instanceof Pose) {
			Pose p = (Pose) o;
			return super.equals(p) && this.yaw == p.yaw;
		} else
			return false;
	}

	/**
	 * Adds this {@code Pose} and the specified {@code Pose}.
	 * 
	 * @param other
	 *            a {@code Pose}
	 * @return the {@code Pose} resulting from adding this {@code Pose} and the specified {@code Pose}
	 */
	public Pose add(Pose other) {
		Position o = other.rotate(yaw + other.yaw);
		return new Pose(x + o.x, y + o.y, yaw + other.yaw);
	}

	/**
	 * Returns the average of the specified {@code Pose}s.
	 * 
	 * @param poses
	 *            {@code Pose}s
	 * @return the average of the specified {@code Pose}s
	 */
	public static Pose average(Pose... poses) {
		if (poses == null || poses.length == 0)
			return null;
		double x = 0;
		double y = 0;
		double yaw = 0;
		int count = 0;
		for (var pose : poses) {
			if (pose != null) {
				x += pose.x;
				y += pose.y;
				yaw += pose.yaw;
				count++;
			}
		}
		if (count == 0)
			return null;
		return new Pose(x / poses.length, y / poses.length, yaw / poses.length);
	}

	/**
	 * Returns the {@code Pose} resulting from moving this {@code Pose} forward in its current direction by the
	 * specified magnitude.
	 * 
	 * @param magnitude
	 *            the magnitude of movement
	 * @return the {@code Pose} resulting from moving this {@code Pose} forward in its current direction by the
	 *         specified magnitude
	 */
	public Pose move(double magnitude) {
		return new Pose(x + magnitude * Math.cos(yaw), y + magnitude * Math.sin(yaw), yaw);
	}

	/**
	 * Returns the {@code Pose} resulting from moving this {@code Pose} according to the transformation that moves the
	 * first reference {@code Pose} to the second reference {@code Pose}.
	 * 
	 * @param r1
	 *            the first reference {@code Pose}
	 * @param r2
	 *            the second reference {@code Pose}
	 * @return the {@code Pose} resulting from moving this {@code Pose} according to the transformation that moves the
	 *         first reference {@code Pose} to the second reference {@code Pose}
	 */
	public Pose move(Pose r1, Pose r2) {
		double angle = r2.yawInRadians() - r1.yawInRadians();
		Position translation = r1.displacementTo(r2).rotate(-r1.yaw);
		// System.out.println(r2 + " vs. "
		// + new Pose(translation.rotate(r1.yawInRadians()).translate(r1), r1.yawInRadians() + angle));
		return new Pose(translation.rotate(this.yawInRadians()).translate(this), this.yawInRadians() + angle);
	}

	/**
	 * Normalizes the specified angle in radians to the range (-Math.PI, Math.PI].
	 * 
	 * @param angleInRadians
	 *            an angle in radians
	 * @return the normalized angle in the range (-Math.PI, Math.PI]
	 */
	public static double normalize(double angleInRadians) {
		if (-Math.PI < angleInRadians && angleInRadians <= Math.PI)
			return angleInRadians;
		angleInRadians -= Math.floor(angleInRadians / (2 * Math.PI)) * (2 * Math.PI);
		return angleInRadians > Math.PI ? angleInRadians - 2 * Math.PI : angleInRadians;
	}

	/**
	 * Determines whether or not any coordinate or yaw value of this {@code Pose} is NaN.
	 * 
	 * @return {@code true} if any coordinate or yaw value of this {@code Pose} is NaN; {@code false} otherwise
	 */
	public boolean hasNaN() {
		return Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(yaw);
	}

}