// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.aster;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.aster.commands.drive.TurnCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.aster.subsystems.DriveSubsystem;
import frc.common.PoseEstimationSubsystem;
import frc.aster.Constants.ControllerConstants;
import frc.aster.commands.drive.DriveDistanceCommand;
import hlib.drive.Pose;

import hlib.drive.Position;

public class RobotContainer implements frc.common.RobotContainer {
	DriveSubsystem m_driveSubsystem = new DriveSubsystem();

	PoseEstimationSubsystem m_poseEstimationSubsystem = new PoseEstimationSubsystem() {
		{
			add(() -> {
				var pose = toPose(DriveSubsystem.get().getPose());
				visionTable.getEntry("odometry").setString(String.format(
						"pose: %s, left encoder position: %.1f, right encoder position: %.1f, yaw: %.1f degrees",
						pose.toString(),
						DriveSubsystem.get().getLeftEncoderPosition(),
						DriveSubsystem.get().getRightEncoderPosition(),
						Math.toRadians(DriveSubsystem.get().getHeading())));
				return pose;
			});
		}

	};

	Joystick m_operatorController = new Joystick(ControllerConstants.kOperatorControllerPort);

	public RobotContainer() {
		configureButtonBindings();
	}

	private void configureButtonBindings() {
		// new JoystickButton(m_operatorController,
		// ControllerConstants.Button.kTriangle)
		// .whileTrue(new TurnCommand(30));
		var target = new Position(6.809, -3.859);
		Supplier<Double> turnSupplier = () -> {
			Pose pose = m_poseEstimationSubsystem.estimatedPose();
			if (pose != null)
				return pose.displacementTo(target).angleInDegrees() - pose.yawInDegrees();
			return 0.0;
		};
		Supplier<Double> driveSupplier = () -> {
			Pose pose = m_poseEstimationSubsystem.estimatedPose();
			if (pose != null)
				return 1 - pose.distance(target);
			return 0.0;
		};
		new JoystickButton(m_operatorController, ControllerConstants.Button.kTriangle)
				.whileTrue(new TurnCommand(turnSupplier, 2)
						.andThen(new DriveDistanceCommand(driveSupplier, 0.1)));
	}

	public Command getAutonomousCommand() {
		return null;
	}
}
