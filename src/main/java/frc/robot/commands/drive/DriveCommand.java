package frc.robot.commands.drive;

import static frc.robot.Constants.DriveConstants.*;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

/**
 * The {@code DriveCommand} is responsible for moving the robot from the current
 * pose to a certain target pose.
 * It utilizes three {@code ProfiledPIDController}s to precisely control the
 * robot in the x, y, and yaw dimensions.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public class DriveCommand extends Command {

	/**
	 * The {@code DriveSubsystem} used by this {@code DriveCommand}.
	 */
	private DriveSubsystem m_driveSubsystem;

	/**
	 * The {@code Supplier<Pose2d>} that calculates the target pose to which the
	 * robot should move.
	 * This is used at the commencement of this {@code DriveCommand} (i.e.,
	 * when the scheduler begins to periodically execute this {@code
	 * DriveCommand}).
	 */
	private Supplier<Pose2d> m_targetPoseSupplier;

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
	 * Constructs a new {@code DriveCommand} whose purpose is to move the
	 * robot by the specified x and y-coordinate and yaw values in the
	 * robot-oriented fashion.
	 * 
	 * @param driveSubsystem    the {@code DriveSubsystem} to use
	 * @param changeX           the desired change in the x-coordinate value
	 * @param changeY           the desired change in the y-coordinate value
	 * @param changeYaw         the desired change in the yaw value
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance    the angle error in degrees which is tolerable
	 */
	public DriveCommand(DriveSubsystem driveSubsystem, double changeX, double changeY, double changeYaw,
			double distanceTolerance,
			double angleTolerance) {
		this(driveSubsystem, () -> {
			var target = new Transform2d(new Translation2d(changeX, changeY), Rotation2d.fromDegrees(changeYaw));
			return driveSubsystem.getPose().plus(target);
		}, distanceTolerance, angleTolerance);
	}

	/**
	 * Constructs a new {@code DriveCommand} whose purpose is to move the
	 * robot to a certain target pose.
	 * 
	 * @param driveSubsystem     the {@code DriveSubsystem} to use
	 * @param targetPoseSupplier a {@code Supplier<Pose2d>} that provides the
	 *                           target pose to which the robot should move.
	 *                           This is used at the commencement of this
	 *                           {@code DriveCommand} (i.e., when the scheduler
	 *                           begins to periodically execute this
	 *                           {@code DriveCommand})
	 * @param distanceTolerance  the distance error in meters which is tolerable
	 * @param angleTolerance     the angle error in degrees which is tolerable
	 */
	public DriveCommand(DriveSubsystem driveSubsystem, Supplier<Pose2d> targetPoseSupplier, double distanceTolerance,
			double angleTolerance) {
		m_driveSubsystem = driveSubsystem;
		m_targetPoseSupplier = targetPoseSupplier;
		var constraints = new TrapezoidProfile.Constraints(kDriveMaxVelocity, kDriveMaxAcceleration);
		m_controllerX = new ProfiledPIDController(kDriveP, kDriveI, kDriveD, constraints);
		m_controllerY = new ProfiledPIDController(kDriveP, kDriveI, kDriveD, constraints);
		m_controllerYaw = new ProfiledPIDController(kTurnP, kTurnI, kTurnD,
				new TrapezoidProfile.Constraints(kTurnMaxVelocity, kTurnMaxAcceleration));
		m_controllerX.setTolerance(distanceTolerance);
		m_controllerY.setTolerance(distanceTolerance);
		m_controllerYaw.setTolerance(angleTolerance);
		m_controllerYaw.enableContinuousInput(-180, 180);
		addRequirements(m_driveSubsystem);
	}

	/**
	 * Is invoked at the commencement of this {@code DriveCommand} (i.e,
	 * when the scheduler begins to periodically execute this {@code DriveCommand}).
	 */
	@Override
	public void initialize() {
		Pose2d pose = m_driveSubsystem.getPose();
		var targetPose = pose;
		try {
			targetPose = m_targetPoseSupplier.get();
		} catch (Exception e) {
		}
		m_controllerX.reset(pose.getX());
		m_controllerY.reset(pose.getY());
		m_controllerYaw.reset(pose.getRotation().getDegrees());
		m_controllerX.setGoal(targetPose.getX());
		m_controllerY.setGoal(targetPose.getY());
		m_controllerYaw.setGoal(targetPose.getRotation().getDegrees());
	}

	/**
	 * Is invoked periodically by the scheduler until this
	 * {@code DriveCommand} is either ended or interrupted.
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
	 * Is invoked once this {@code DriveCommand} is either ended or interrupted.
	 * 
	 * @param interrupted indicates if this {@code DriveCommand} was interrupted
	 */
	@Override
	public void end(boolean interrupted) {
		m_driveSubsystem.stopDriving();
	}

	/**
	 * Determines whether or not this {@code DriveCommand} needs to end.
	 * 
	 * @return {@code true} if this {@code DriveCommand} needs to end;
	 *         {@code false} otherwise
	 */
	@Override
	public boolean isFinished() {
		return m_controllerX.atGoal() && m_controllerY.atGoal() && m_controllerYaw.atGoal();
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

	/**
	 * Constructs a {@code Commmand} for algining the robot to the specified
	 * {@code Pose}.
	 * 
	 * @param targetPose         the target pose
	 * @param distanceTolerance  the distance error in meters which is tolerable
	 * @param angleTolerance     the angle error in degrees which is tolerable
	 * @param driveSubsystem     the {@code DriveSubsystem} to use
	 * @param limeLightSubsystem the {@code LimeLightSubsystem} to use
	 * @return a {@code Commmand} for algining the robot to the specified
	 *         {@code Pose}
	 */
	public static Command alignTo(Pose2d targetPose, double distanceTolerance, double angleTolerance,
			DriveSubsystem driveSubsystem,
			LimeLightSubsystem limeLightSubsystem) {
		return new DriveCommand(driveSubsystem,
				() -> driveSubsystem.getPose().plus(limeLightSubsystem.transformationTo(targetPose)),
				distanceTolerance, angleTolerance);
	}

	/**
	 * Constructs a {@code Commmand} for turning the robot to the specified target
	 * position.
	 * 
	 * @param targetPosition     the target position
	 * @param distanceTolerance  the distance error in meters which is tolerable
	 * @param angleTolerance     the angle error in degrees which is tolerable
	 * @param driveSubsystem     the {@code DriveSubsystem} to use
	 * @param limeLightSubsystem the {@code LimeLightSubsystem} to use
	 * @return a {@code Commmand} for turning the robot to the specified target
	 *         position
	 */
	public static Command turnTo(Translation2d targetPosition,
			double distanceTolerance,
			double angleTolerance, DriveSubsystem driveSubsystem,
			LimeLightSubsystem limeLightSubsystem) {
		return new DriveCommand(driveSubsystem,
				() -> driveSubsystem.getPose()
						.plus(limeLightSubsystem.transformationToward(targetPosition)),
				distanceTolerance, angleTolerance);
	}

	/**
	 * Constructs a {@code Commmand} for moving the robot toward the specified
	 * target
	 * position while ensuring that the robot is away from the target by the
	 * specified distance.
	 * 
	 * @param targetPosition     the target position
	 * @param distanceToTarget   the desired distance between the robot and the
	 *                           target position
	 * @param distanceTolerance  the distance error in meters which is tolerable
	 * @param angleTolerance     the angle error in degrees which is tolerable
	 * @param driveSubsystem     the {@code DriveSubsystem} to use
	 * @param limeLightSubsystem the {@code LimeLightSubsystem} to use
	 * @return a {@code Commmand} for turning the robot to the specified target
	 *         position
	 */
	public static Command moveToward(Translation2d targetPosition, double distanceToTarget,
			double distanceTolerance,
			double angleTolerance, DriveSubsystem driveSubsystem,
			LimeLightSubsystem limeLightSubsystem) {
		return new DriveCommand(driveSubsystem,
				() -> driveSubsystem.getPose()
						.plus(limeLightSubsystem.transformationToward(targetPosition, distanceToTarget)),
				distanceTolerance, angleTolerance);
	}

}