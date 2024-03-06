package frc.aster.commands.drive;

import java.util.function.Supplier;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * The {@code DriveCommand} is responsible for moving the robot from the current
 * pose to a certain target pose.
 * It utilizes three {@code ProfiledPIDController}s to precisely control the
 * robot in the x, y, and yaw dimensions.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public class DriveCommand extends SequentialCommandGroup {

	/**
	 * Constructs a new {@code DriveCommand} whose purpose is to move the
	 * robot to a certain target.
	 * 
	 * @param targetPose
	 *                          the target pose whose x and y-coordinate values are
	 *                          in meters and yaw value is in degrees
	 * @param distanceTolerance
	 *                          the distance error in meters which is tolerable
	 * @param angleTolerance
	 *                          the angle error in degrees which is tolerable
	 */
	public DriveCommand(Transform2d targetPose, double distanceTolerance, double angleTolerance) {
		this(() -> targetPose, distanceTolerance, angleTolerance);
	}

	/**
	 * Constructs a new {@code DriveCommand} whose purpose is to navigate the robot
	 * towards the specified target and stop at the specified distance
	 * away from the target.
	 * 
	 * @param currentPose       the current {@code Pose2d} of the robot
	 * @param targetPosition    the target position whose x and y-coordinate values
	 *                          are in meters
	 * @param distanceToTarget  the desired distance to the target
	 * @param distanceTolerance
	 *                          the distance error in meters which is tolerable
	 * @param angleTolerance
	 *                          the angle error in degrees which is tolerable
	 */
	public DriveCommand(Translation2d targetPosition, double distanceToTarget,
			double distanceTolerance,
			double angleTolerance) {
		this(() -> {
			// var currentPose = DriveSubsystem.get().getPose();
			Transform2d transform = null;
			// PoseEstimationSubsystem.get().recordPose("Target@Odomoetry", targetPose);
			SmartDashboard.putString("transform", "" + transform);
			return transform;
		},
				distanceTolerance,
				angleTolerance);
	}

	/**
	 * Constructs a new {@code DriveCommand} whose purpose is to move the
	 * robot to a certain target pose.
	 * 
	 * @param targetPoseCalculator
	 *                             a {@code Supplier<Pose2d>} that calculates the
	 *                             target pose to which the robot should move.
	 *                             This is used at the commencement of this {@code
	 *                             DriveCommand} (i.e.,
	 *                             when the scheduler begins to periodically execute
	 *                             this {@code
	 *                             DriveCommand})
	 * 
	 * @param distanceTolerance    the distance error in
	 *                             meters which is
	 *                             tolerable
	 * @param angleTolerance
	 *                             the angle error in degrees which is tolerable
	 */
	public DriveCommand(Supplier<Transform2d> targetPoseCalculator, double distanceTolerance, double angleTolerance) {
		this(() -> Math.abs(targetPoseCalculator.get().getRotation().getDegrees()) < 90
				? targetPoseCalculator.get().getTranslation().getNorm()
				: -targetPoseCalculator.get().getTranslation().getNorm(),
				() -> targetPoseCalculator.get().getRotation().getDegrees(), distanceTolerance, angleTolerance);
	}

	/**
	 * Constructs a new {@code DriveCommand} whose purpose is to move the
	 * robot to a certain target pose.
	 * 
	 * @param targetPoseCalculator
	 *                             a {@code Supplier<Pose2d>} that calculates the
	 *                             target pose to which the robot should move.
	 *                             This is used at the commencement of this {@code
	 *                             DriveCommand} (i.e.,
	 *                             when the scheduler begins to periodically execute
	 *                             this {@code
	 *                             DriveCommand})
	 * 
	 * @param distanceTolerance    the distance error in
	 *                             meters which is
	 *                             tolerable
	 * @param angleTolerance
	 *                             the angle error in degrees which is tolerable
	 */
	public DriveCommand(Supplier<Double> targetDistanceCalculator, Supplier<Double> targetAngleCalculator,
			double distanceTolerance, double angleTolerance) {
		addCommands(new TurnCommand(targetAngleCalculator, angleTolerance));
		addCommands(new DriveDistanceCommand(targetDistanceCalculator, distanceTolerance));
	}

	/**
	 * Constructs a {@code SequentialCommandGroup} for passing through all of the
	 * specified
	 * {@code Pose2d}s.
	 * 
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance
	 *                          the angle error in degrees which is tolerable
	 * @param poses             the {@code Pose2d}s to pass through
	 * @return a {@code SequentialCommandGroup} for passing through all of the
	 *         specified {@code Pose2d}s
	 */
	public static Command createCommand(double distanceTolerance, double angleTolerance, Pose2d... poses) {
		if (poses == null || poses.length == 0)
			return new Command() {
				public boolean isFinished() {
					return true;
				}
			};
		Command c = null;
		for (var pose : poses) {
			Translation2d p2 = pose.getTranslation();
			c = c == null
					? new DriveCommand(p2, 0.0, distanceTolerance, angleTolerance)
					: c.andThen(new DriveCommand(p2, 0.0, distanceTolerance, angleTolerance));
		}
		return c;
	}

}