package frc.robot.commands.drive;

import static frc.robot.Constants.DriveConstants.*;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

/**
 * The {@code DrivePathCommand} is responsible for moving the robot from
 * the current pose to a series of target poses.
 * It utilizes three {@code ProfiledPIDController}s to precisely control the
 * robot in the x, y, and yaw dimensions.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public class DrivePathCommand extends Command {

	/**
	 * The {@code DriveSubsystem} used by this {@code DrivePathCommand}.
	 */
	private DriveSubsystem m_driveSubsystem;

	/**
	 * The {@code Supplier<List<Pose2d>>} that calculates the series of target poses
	 * that the robot should pass through.
	 * This is used at the commencement of this {@code DrivePathCommand} (i.e.,
	 * when the scheduler begins to periodically execute this {@code
	 * DrivePathCommand}).
	 */
	private Supplier<List<Pose2d>> m_pathSupplier;

	/**
	 * The target poses that the robot should pass through.
	 */
	private List<Pose2d> m_targetPoses;

	/**
	 * The index of the current target pose.
	 */
	private int m_current = 0;

	/**
	 * The distance error in meters which is tolerable.
	 */
	private double m_distanceTolerance;

	/**
	 * The angle error in degrees which is tolerable.
	 */
	private double m_angleTolerance;

	/**
	 * The ratio of tolerance at intermediate poses compared to the tolerance at the
	 * final pose.
	 */
	private double m_intermediateTolerance = 5;

	/**
	 * The {@code ProfiledPIDController} for controlling the robot in the x
	 * dimension in meters.
	 */
	private ProfiledPIDController m_controllerX;

	/**
	 * The {@code ProfiledPIDController} for controlling the robot in the y
	 * dimension in meters.
	 */
	private ProfiledPIDController m_controllerY;

	/**
	 * The {@code ProfiledPIDController} for controlling the robot in the yaw
	 * dimension in angles.
	 */
	private ProfiledPIDController m_controllerYaw;

	/**
	 * Constructs a new {@code DrivePathCommand} whose purpose is to move the robot
	 * from the current pose to a series of target poses.
	 * 
	 * @param driveSubsystem     the {@code DriveSubsystem} to use
	 * @param targetPoseSupplier a {@code Supplier<List<Pose2d>>} that calculates
	 *                           the series of target poses that the robot should
	 *                           pass through.
	 *                           This is used at the commencement of this
	 *                           {@code DrivePathCommand} (i.e.,
	 *                           when the scheduler begins to periodically execute
	 *                           this {@code DrivePathCommand})
	 * @param distanceTolerance  the distance error in meters which is tolerable
	 * @param angleTolerance     the angle error in degrees which is tolerable
	 */
	public DrivePathCommand(DriveSubsystem driveSubsystem, Supplier<List<Pose2d>> pathSupplier,
			double distanceTolerance,
			double angleTolerance) {
		m_driveSubsystem = driveSubsystem;
		m_distanceTolerance = distanceTolerance;
		m_angleTolerance = angleTolerance;
		m_pathSupplier = pathSupplier;
		var constraints = new TrapezoidProfile.Constraints(kDriveMaxVelocity, kDriveMaxAcceleration);
		m_controllerX = new ProfiledPIDController(kDriveP, kDriveI, kDriveD, constraints);
		m_controllerY = new ProfiledPIDController(kDriveP, kDriveI, kDriveD, constraints);
		m_controllerYaw = new ProfiledPIDController(kTurnP, kTurnI, kTurnD,
				new TrapezoidProfile.Constraints(kTurnMaxVelocity, kTurnMaxAcceleration));
		m_controllerYaw.enableContinuousInput(-180, 180);
		addRequirements(m_driveSubsystem);
	}

	/**
	 * Is invoked at the commencement of this {@code DrivePathCommand} (i.e,
	 * when the scheduler begins to periodically execute this
	 * {@code DrivePathCommand}).
	 */
	@Override
	public void initialize() {
		m_current = 0;
		Pose2d pose = m_driveSubsystem.getPose();
		m_targetPoses = List.of(pose);
		try {
			var targetPoses = m_pathSupplier.get();
			if (targetPoses != null && targetPoses.size() > 0)
				m_targetPoses = targetPoses;
		} catch (Exception e) {
		}
		m_controllerX.reset(pose.getX());
		m_controllerY.reset(pose.getY());
		m_controllerYaw.reset(pose.getRotation().getDegrees());
		setControllers();
	}

	/**
	 * Is invoked periodically by the scheduler until this
	 * {@code DrivePathCommand} is either ended or interrupted.
	 */
	@Override
	public void execute() {
		Pose2d pose = m_driveSubsystem.getPose();
		double speedX = m_controllerX.calculate(pose.getX());
		double speedY = m_controllerY.calculate(pose.getY());
		// NEGATION if positive turnSpeed: clockwise rotation
		double speedYaw = -m_controllerYaw.calculate(pose.getRotation().getDegrees());
		// speedX = applyThreshold(speedX, DriveConstants.kMinSpeed);
		// speedY = applyThreshold(speedY, DriveConstants.kMinSpeed);
		m_driveSubsystem.setModuleStates(speedX, speedY, speedYaw, true);
	}

	/**
	 * Is invoked once this {@code DrivePathCommand} is either ended or interrupted.
	 * 
	 * @param interrupted indicates if this {@code DrivePathCommand} was interrupted
	 */
	@Override
	public void end(boolean interrupted) {
		m_driveSubsystem.stopDriving();
	}

	/**
	 * Determines whether or not this {@code DrivePathCommand} needs to end.
	 * 
	 * @return {@code true} if this {@code DrivePathCommand} needs to end;
	 *         {@code false} otherwise
	 */
	@Override
	public boolean isFinished() {
		boolean atGoal = m_controllerX.atGoal() && m_controllerY.atGoal() && m_controllerYaw.atGoal();
		if (atGoal && m_current < m_targetPoses.size() - 1) {
			m_current++;
			setControllers();
			return false;
		}
		return atGoal;
	}

	/**
	 * Sets the {@code PIDController}s to move to the next target pose.
	 */
	private void setControllers() {
		var targetPose = m_targetPoses.get(m_current);
		m_controllerX.setGoal(targetPose.getX());
		m_controllerY.setGoal(targetPose.getY());
		m_controllerYaw.setGoal(targetPose.getRotation().getDegrees());
		if (m_current < m_targetPoses.size() - 1) {
			m_controllerX.setTolerance(m_intermediateTolerance * m_distanceTolerance);
			m_controllerY.setTolerance(m_intermediateTolerance * m_distanceTolerance);
			m_controllerYaw.setTolerance(m_intermediateTolerance * m_angleTolerance);
		} else {
			m_controllerX.setTolerance(m_distanceTolerance);
			m_controllerY.setTolerance(m_distanceTolerance);
			m_controllerYaw.setTolerance(m_angleTolerance);
		}
	}

	/**
	 * Constructs a {@code Commmand} for moving the robot to the specified series
	 * of {@code Pose}s.
	 * 
	 * @param targetPoses        the target poses
	 * @param distanceTolerance  the distance error in meters which is tolerable
	 * @param angleTolerance     the angle error in degrees which is tolerable
	 * @param driveSubsystem     the {@code DriveSubsystem} to use
	 * @param limeLightSubsystem the {@code LimeLightSubsystem} to use
	 * @return a {@code Commmand} for moving the robot to the specified series
	 *         of {@code Pose}s.
	 */
	public static Command passThrough(List<Pose2d> targetPoses, double distanceTolerance, double angleTolerance,
			DriveSubsystem driveSubsystem,
			LimeLightSubsystem limeLightSubsystem) {
		Supplier<List<Pose2d>> s = () -> targetPoses.stream()
				.map(targetPose -> driveSubsystem.getPose().plus(limeLightSubsystem.transformationTo(targetPose)))
				.toList();
		return new DrivePathCommand(driveSubsystem, s,
				distanceTolerance, angleTolerance);
	}

}