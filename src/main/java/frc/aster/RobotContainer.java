// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.aster;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.aster.Constants.ControllerConstants;
import frc.aster.Constants.ControllerConstants.Button;
import frc.aster.commands.drive.DefaultDriveCommand;
import frc.aster.commands.drive.DriveCommand;
import frc.aster.commands.drive.DriveDistanceCommand;
import frc.aster.commands.drive.TurnCommand;
import frc.aster.subsystems.DriveSubsystem;
import frc.robot.subsystems.AdvantageScopeUtil;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.LimeLightSubsystem.Pose;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystemAdvanced;

public class RobotContainer implements frc.common.RobotContainer {
	private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private final PoseEstimationSubsystemAdvanced m_poseEstimationSubsystem = new PoseEstimationSubsystemAdvanced();
	private final CommandGenericHID m_controller = new CommandGenericHID(ControllerConstants.kDriverControllerPort);

	private final LimeLightSubsystem m_limeLightSubsystem = new PoseEstimationSubsystem() {
		{
			addPoseSupplier("BotPose@Odometry", () -> m_driveSubsystem.getPose());
		}

		protected NetworkTable table = NetworkTableInstance.getDefault().getTable("AdvantageScope");

		@Override
		public void periodic() {
			super.periodic();
			var pose = estimatedPose();
			table.getEntry("Pose Estimated").setString("" + pose);
			if (pose != null)
				table.getEntry("BotPose'").setDoubleArray(AdvantageScopeUtil.toPose2DAdvantageScope(pose));
			pose = m_driveSubsystem.getPose();
			table.getEntry("BotPose@Odometry").setDoubleArray(AdvantageScopeUtil.toPose2DAdvantageScope(pose));
			try {
				pose = new Pose(m_botpose[0], m_botpose[1], m_botpose[5]);
				table.getEntry("BotPose").setDoubleArray(AdvantageScopeUtil.toPose2DAdvantageScope(pose));
			} catch (Exception e) {
			}
		}

		// public void recordPose(String entryName, Pose2d value) {
		// if (value != null && !(value instanceof Pose))
		// value = new Pose(value.getX(), value.getY(),
		// value.getRotation().getDegrees());
		// if (value == null)
		// table.getEntry(entryName).setDoubleArray(new double[0]);
		// else
		// table.getEntry(entryName).setDoubleArray(AdvantageScopeUtil.toPose2DAdvantageScope(value));
		// }

		// public void recordString(String entryName, String value) {
		// table.getEntry(entryName).setString(value);
		// }

	};

	public RobotContainer() {
		m_poseEstimationSubsystem.addPoseSupplier("BotPose@Odometry",
				() -> m_driveSubsystem.getPose());
		configureButtonBindings();
	}

	private void configureButtonBindings() {
		m_controller.button(Button.kSquare).whileTrue(new DriveDistanceCommand(2.0, 0.1));
		m_controller.button(Button.kTriangle).whileTrue(new DriveDistanceCommand(-2.0, 0.1));
		m_controller.button(Button.kCircle).whileTrue(new TurnCommand(30, 2));
		// var target = new Translation2d(6.809, -3.859);
		Supplier<Double> turnSupplier = () -> {
			Map<Integer, Rotation2d> m = m_poseEstimationSubsystem.getRotationsToDetectedTags();
			if (m.size() > 0)
				return m.values().iterator().next().getDegrees();
			return 0.0;
		};
		Supplier<Double> driveSupplier = () -> {
			Map<Integer, Double> m = m_poseEstimationSubsystem.getDistancesToDetectedTags();
			if (m.size() > 0)
				return 1 - m.values().iterator().next();
			return 0.0;
		};
		Command[] samples = {
				new TurnCommand(30, 3)
						.andThen(new TurnCommand(-30, 3)),
				new TurnCommand(new Translation2d(8.308467, 1.442593),
						m_limeLightSubsystem,
						3),
				new TurnCommand(turnSupplier, 2)
						.andThen(new DriveDistanceCommand(driveSupplier, 0.1)),
				new DriveDistanceCommand(1, 0.2)
						.andThen(new DriveDistanceCommand(-1, 0.2)),
				new DriveDistanceCommand(new Translation2d(8.308467, 1.442593), 1.2,
						m_limeLightSubsystem, 0.2),
				new TurnCommand(new Translation2d(8.308467, 1.442593),
						m_limeLightSubsystem,
						3).andThen(
								new DriveDistanceCommand(new Translation2d(8.308467, 1.442593), 1.2,
										m_limeLightSubsystem, 0.2)),
				DriveCommand.createCommand(0.05, 1,
						new Pose(1.0, 0, 0),
						new Pose(0, 0, 0)),
				DriveCommand.createCommand(0.05, 1,
						new Pose(0, 0, 0),
						new Pose(1, 1.0, 0),
						new Pose(1.44, 1.5, 90),
						new Pose(1.44, 1.7, 90),
						new Pose(-0.8, -0.5, -120),
						new Pose(0, 0, 0)),
				DriveCommand.createCommand(0.05, 1,
						new Pose(6.85, 3.0, 0),
						new Pose(6, 3.0, 0),
						new Pose(6.44, 3.5, 90),
						new Pose(6.44, 3.7, 90),
						new Pose(4.2, 1.5, -120),
						new Pose(6.85, 3.0, 0)) };
		m_controller.button(Button.kX).whileTrue(samples[0]);
		m_driveSubsystem.setDefaultCommand(new DefaultDriveCommand(
				() -> -m_controller.getRawAxis(ControllerConstants.Axis.kLeftY),
				() -> m_controller.getRawAxis(ControllerConstants.Axis.kLeftTrigger),
				() -> m_controller.getRawAxis(ControllerConstants.Axis.kRightTrigger)));
	}

	public Command getAutonomousCommand() {
		return null;
	}
}