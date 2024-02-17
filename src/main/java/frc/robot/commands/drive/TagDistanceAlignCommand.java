package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;

/**
 * The {@code TagDistanceAlignCommand} moves the robot so that the robot faces
 * toward the primary in-view
 * AprilTag and is away from the AprilTag by a specified distance.
 * 
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 */
public class TagDistanceAlignCommand extends DriveCommand {

	/**
	 * Constructs a {@code TagDistanceAlignCommand}.
	 * 
	 * @param driveSubsystem    the {@code DriveSubsystem} used by the
	 *                          {@code TagDistanceAlignCommand}
	 * @param distanceToTag
	 *                          the target distance in meters between the robot and
	 *                          the primary in-view AprilTag
	 * @param distanceTolerance
	 *                          the distance error in meters which is tolerable
	 * @param angleTolerance
	 *                          the angle error in degrees which is tolerable
	 */
	public TagDistanceAlignCommand(DriveSubsystem driveSubsystem, double distanceToTag, double distanceTolerance,
			double angleTolerance) {
		super(driveSubsystem, () -> {
			var transform = TagAlignCommand.transformationToTagPosition();
			var translation = transform.getTranslation();
			double norm = translation.getNorm();
			if (norm == 0)
				return driveSubsystem.getPose();
			translation = translation.times(1 - distanceToTag / norm);
			var targetPose = driveSubsystem.getPose().plus(new Transform2d(translation, transform.getRotation()));
			SmartDashboard.putString(
					"drive",
					String.format("transform to tag: %s, current pose: %s, target pose: %s", "" + transform,
							"" + driveSubsystem.getPose(), "" + targetPose));
			return targetPose;
		}, distanceTolerance, angleTolerance);
	}

}