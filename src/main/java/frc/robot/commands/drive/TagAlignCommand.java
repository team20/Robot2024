package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * The {@code TagAlignCommand} rotates the robot to the primary in-view
 * AprilTag. It utilizes a {@code ProfiledPIDController} to maintain
 * precision in the rotational movement.
 * 
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 */
public class TagAlignCommand extends Command {

	/**
	 * The {@code DriveSubsystem} used by this {@code TagAlignCommand}.
	 */
	private DriveSubsystem m_driveSubsystem;

	/**
	 * The {@code ProfiledPIDController} for controlling the rotational movement.
	 */
	private ProfiledPIDController m_turnController;

	/**
	 * Constructs a {@code TagAlignCommand}.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} used by the
	 *                       {@code TagAlignCommand}
	 * @param angleTolerance
	 *                       the angle error in degrees which is tolerable
	 */
	public TagAlignCommand(DriveSubsystem driveSubsystem, double angleTolerance) {
		m_driveSubsystem = driveSubsystem;
		var constraints = new TrapezoidProfile.Constraints(DriveConstants.kTurnMaxVelocity,
				DriveConstants.kTurnMaxAcceleration);
		m_turnController = new ProfiledPIDController(DriveConstants.kTurnP, DriveConstants.kTurnI,
				DriveConstants.kTurnD,
				constraints);
		m_turnController.setTolerance(angleTolerance);
		m_turnController.enableContinuousInput(-180, 180);
		addRequirements(m_driveSubsystem);
	}

	/**
	 * Is invoked at the commencement of this {@code TagAlignCommand} (i.e, when the
	 * scheduler begins to periodically execute this {@code TagAlignCommand}).
	 */
	@Override
	public void initialize() {
		// double heading = -m_driveSubsystem.getHeading().getDegrees(); // NEGATION if
		// the heading data is negated
		double heading = m_driveSubsystem.getPose().getRotation().getDegrees();
		double goal = heading;
		var t = transformationToTagPosition();
		if (t != null)
			goal += t.getRotation().getDegrees();
		m_turnController.reset(heading);
		m_turnController.setGoal(goal);
	}

	/**
	 * Finds the transformation that maps the current robot pose to the the primary
	 * in-view AprilTag.
	 * 
	 * @return the transformation that maps the current robot pose to the mid point
	 *         of the detected AprilTags
	 */
	public static Transform2d transformationToTagPosition() {
		var a = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_robotspace")
				.getDoubleArray(new double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
		var tagPosition = new Translation2d(a[0], a[2]).rotateBy(Rotation2d.fromDegrees(-90));
		return tagPosition.getNorm() == 0 ? null : new Transform2d(tagPosition, tagPosition.getAngle());
	}

	/**
	 * Is invoked periodically by the scheduler until this {@code TagAlignCommand}
	 * is either ended or interrupted.
	 */
	@Override
	public void execute() {
		// double heading = -m_driveSubsystem.getHeading().getDegrees(); // NEGATION if
		// the heading data is negated
		double heading = m_driveSubsystem.getPose().getRotation().getDegrees();
		double turnSpeed = m_turnController.calculate(heading);
		turnSpeed = -turnSpeed; // NEGATION if positive turnSpeed = clockwise rotation
		// turnSpeed = applyThreshold(turnSpeed, DriveConstants.kMinSpeed); // for
		// convergence
		m_driveSubsystem.setModuleStates(0, 0, turnSpeed, true);
	}

	/**
	 * Is invoked once this {@code TagAlignCommand} is either ended or interrupted.
	 * 
	 * @param interrupted
	 *                    indicates if this {@code TagAlignCommand} was interrupted
	 */
	@Override
	public void end(boolean interrupted) {
		m_driveSubsystem.setModuleStates(0, 0, 0, true);
	}

	/**
	 * Determines whether or not this {@code TagAlignCommand} needs to end.
	 * 
	 * @return {@code true} if this {@code TagAlignCommand} needs to end;
	 *         {@code false} otherwise
	 */
	@Override
	public boolean isFinished() {
		return m_turnController.atGoal();
	}

	/**
	 * Applies the specified threshold to the specified value.
	 * 
	 * @param value     the value to be thresholded
	 * @param threshold the threshold limit
	 * @return the original value if the absolute value of that value is greater or
	 *         equal to the threshold; the threshold with the original value's sign
	 *         otherwise
	 */
	public static double applyThreshold(double value, double threshold) {
		return Math.abs(value) < threshold ? Math.signum(value) * threshold : value;
	}

}