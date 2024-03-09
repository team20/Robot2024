// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ControllerConstants.Axis;
import frc.robot.Constants.ControllerConstants.Button;
import frc.robot.commands.BangBangDriveDistance;
import frc.robot.commands.PIDTurnCommand;
import frc.robot.commands.SetSteering;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.drive.DriveDistanceCommand;
import frc.robot.commands.drive.TagAlignCommand;
import frc.robot.commands.drive.TurnCommand;
import frc.robot.subsystems.AdvantageScopeUtil;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLightEmulationSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.LimeLightSubsystem.Pose;
import frc.robot.subsystems.PoseEstimationSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer implements frc.common.RobotContainer {

	private final CommandGenericHID m_controller = new CommandGenericHID(ControllerConstants.kDriverControllerPort);
	private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	protected final ArduinoSubsystem m_ArduinoSubsystem = new ArduinoSubsystem();
	private final SendableChooser<Command> m_autoSelector = new SendableChooser<Command>();
	// private final LimeLightSubsystem m_limeLightSubsystem = new
	// LimeLightSubsystem() {
	private final LimeLightSubsystem m_limeLightSubsystem = new PoseEstimationSubsystem() {
		{
			addPoseSupplier("BotPose@Odometry", () -> m_driveSubsystem.getPose());
		}

		@Override
		public void periodic() {
			super.periodic();
			var pose = estimatedPose();
			table.getEntry("Pose Estimated").setString("" + pose);
			if (pose != null)
				table.getEntry("BotPose'").setDoubleArray(AdvantageScopeUtil.toPose2DAdvantageScope(pose));
			table.getEntry("BotPose@Odometry")
					.setDoubleArray(AdvantageScopeUtil.toPose2DAdvantageScope(m_driveSubsystem.getPose()));
			table.getEntry("BotPose'@Odometry")
					.setDoubleArray(AdvantageScopeUtil.toPose2DAdvantageScope(m_driveSubsystem.getPose()));
			try {
				pose = new Pose(m_botpose[0], m_botpose[1], m_botpose[5]);
				table.getEntry("BotPose").setDoubleArray(AdvantageScopeUtil.toPose2DAdvantageScope(pose));
				double d1 = transformationTo(new Pose(-6.44, 3.5, -90)).getTranslation().getNorm();
				double d2 = transformationTo(new Pose(-7.87, 1.45, 0)).getTranslation().getNorm();
				table.getEntry("distances").setString(String.format("%.2f, %.2f", d1, d2));

			} catch (Exception e) {
			}
		}

	};

	protected NetworkTable table = NetworkTableInstance.getDefault().getTable("AdvantageScope");

	public void recordPose(String entryName, Pose2d value) {
		if (value == null)
			table.getEntry(entryName).setDoubleArray(new double[0]);
		else
			table.getEntry(entryName).setDoubleArray(AdvantageScopeUtil.toPose2DAdvantageScope(new Pose(value.getX(),
					value.getY(), value.getRotation().getDegrees())));
	}

	public void recordString(String entryName, String value) {
		table.getEntry(entryName).setString(value);
	}

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		if (RobotBase.isSimulation())
			new LimeLightEmulationSubsystem(new Pose(-2, 0, 180), 0.01, m_driveSubsystem);

		// Configure the button bindings
		m_autoSelector.addOption("Test Steering", SetSteering.getCalibrationCommand(m_driveSubsystem));
		m_autoSelector.addOption("PID Turn 90 degrees", new PIDTurnCommand(m_driveSubsystem, 90, 0.5));
		m_autoSelector.addOption("Bang Bang Drive 2 Meters", new BangBangDriveDistance(m_driveSubsystem, 2, 0.01));
		// m_autoSelector.addOption("PID Drive 2 Meters",
		// DriveDistanceCommand.create(m_driveSubsystem, 3.0, 0.01));
		// m_autoSelector.addOption("Knock Over Blocks",
		// CommandComposer.getBlocksAuto(m_driveSubsystem));

		SmartDashboard.putData(m_autoSelector);
		configureButtonBindings();
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by instantiating a {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
	 * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		// new Trigger(() -> DriverStation.getMatchTime() >= 20)
		// .onTrue(m_ArduinoSubsystem.writeStatus(StatusCode.RAINBOW_PARTY_FUN_TIME));
		m_driveSubsystem.setDefaultCommand(m_driveSubsystem.driveCommand(
				() -> m_controller.getRawAxis(Axis.kLeftY),
				() -> m_controller.getRawAxis(Axis.kLeftX),
				() -> m_controller.getRawAxis(Axis.kRightTrigger),
				() -> m_controller.getRawAxis(Axis.kLeftTrigger)));
		// m_controller.button(Button.kTriangle).onTrue(m_driveSubsystem.resetHeadingCommand());
		// m_controller.button(Button.kTriangle).onTrue(m_driveSubsystem.alignModulesToZeroComamnd().withTimeout(0.5));
		// m_controller.button(Button.kSquare).onTrue(m_driveSubsystem.resetEncodersCommand());
		// m_controller.button(Button.kX).onTrue(new
		// frc.robot.commands.DriveDistanceCommand(m_driveSubsystem, 10, 0.01));
		Command[] samples = {
				new DriveCommand(m_driveSubsystem, 1.0, 0, 0, 0.1, 5)
						.andThen(new DriveCommand(m_driveSubsystem, 0, 1, 0, 0.1, 5))
						.andThen(new DriveCommand(m_driveSubsystem, 0, 0, 90, 0.1, 5))
						.andThen(new DriveCommand(m_driveSubsystem, -1, 1, -90, 0.1, 5)),
				CommandComposer.getFiveScoreBlueAutoCommandOptimized(m_driveSubsystem,
						m_limeLightSubsystem),
				CommandComposer.getFourScoreMiddleBlueAutoCommand(m_driveSubsystem,
						m_limeLightSubsystem),
				CommandComposer.getFourScoreMiddleBlueAutoCommandOptimized(m_driveSubsystem,
						m_limeLightSubsystem),
				CommandComposer.getFourScoreMiddleCenterBlueAutoCommand(m_driveSubsystem,
						m_limeLightSubsystem),
				CommandComposer.getFourScoreMiddleCenterBlueAutoCommandOptimized(m_driveSubsystem,
						m_limeLightSubsystem),
				CommandComposer.getFourScoreMiddleBottomBlueAutoCommand(m_driveSubsystem,
						m_limeLightSubsystem),
				CommandComposer.getFourScoreMiddleBottomBlueAutoCommandOptimized(m_driveSubsystem,
						m_limeLightSubsystem),
				CommandComposer.getAlignToBlueAmpCommand(m_driveSubsystem, m_limeLightSubsystem),
				new DriveCommand(m_driveSubsystem, 1.0, 0, 0, 0.1, 5)
						.andThen(new DriveCommand(m_driveSubsystem, 0, 1, 0, 0.1, 5))
						.andThen(new DriveCommand(m_driveSubsystem, 0, 0, 45, 0.1, 5))
						.andThen(new DriveCommand(m_driveSubsystem, -1, -1, -45, 0.1, 5)),
				new TurnCommand(m_driveSubsystem, 45, 5)
						.andThen(new TurnCommand(m_driveSubsystem, -45, 5)),
				new DriveDistanceCommand(m_driveSubsystem, 1, 0.1)
						.andThen(new DriveDistanceCommand(m_driveSubsystem, -1, 0.1)),
				new TagAlignCommand(m_driveSubsystem, 5),
				new TurnCommand(m_driveSubsystem, 45, 5)
						.andThen(new TurnCommand(m_driveSubsystem, -45, 5))
						.andThen(new DriveDistanceCommand(m_driveSubsystem, 1, 0.1))
						.andThen(new DriveDistanceCommand(m_driveSubsystem, -1, 0.1))
						.andThen(new TurnCommand(m_driveSubsystem, 45, 5))
						.andThen(new DriveDistanceCommand(m_driveSubsystem, 1, 0.1))
						.andThen(new DriveDistanceCommand(m_driveSubsystem, -1, 0.1))
						.andThen(new TurnCommand(m_driveSubsystem, -45, 5))
		};

		// public static Command getFiveScoreBlueAutoCommand(DriveSubsystem
		// driveSubsystem,
		// SimpleVisionSubsystem visionSubsystem, FlywheelSubsystem flywheelSubsystem,
		// AimerSubsystem aimerSubsystem,
		// IndexerSubsystem indexerSubsystem, Targeter targeter,
		// LimeLightSubsystem limeLightSubsystem, IntakeSubsystem intakeSubsystem,
		// PneumaticsSubsystem pneumaticsSubsystem, ArduinoSubsystem arduinoSubsystem)

		m_controller.button(Button.kSquare)
				.whileTrue(CommandComposer.getFiveScoreBlueAutoCommand(m_driveSubsystem,
						null, null, null, null,
						null,
						m_limeLightSubsystem, null, null, null));
		m_controller.button(Button.kX)
				.whileTrue(CommandComposer.getFiveScoreRedAutoCommand(m_driveSubsystem,
						null, null, null, null,
						null,
						m_limeLightSubsystem, null, null, null));
		m_controller.button(Button.kCircle)
				.whileTrue(CommandComposer.getFourScoreTwoMiddleTopBlueAuto(m_driveSubsystem,
						null, null, null, null,
						null,
						m_limeLightSubsystem, null, null, null));
		m_controller.button(Button.kTriangle)
				.whileTrue(
						CommandComposer.getFourScoreTwoMiddleTopRedAuto(m_driveSubsystem, null,
								null, null, null, null,
								m_limeLightSubsystem, null, null, null));

		// m_controller.button(Button.kSquare)
		// .whileTrue(CommandComposer.getThreeScoreOneMiddleTopBlueAuto(m_driveSubsystem,
		// null, null, null, null,
		// null,
		// m_limeLightSubsystem, null, null, null));
		// m_controller.button(Button.kX)
		// .whileTrue(
		// CommandComposer.getThreeScoreOneMiddleTopRedAuto(m_driveSubsystem, null,
		// null, null, null, null,
		// m_limeLightSubsystem, null, null, null));
		// m_controller.button(Button.kSquare)
		// .whileTrue(CommandComposer.getFiveScoreBlueAutoCommand(m_driveSubsystem,
		// null, null, null, null, null,
		// m_limeLightSubsystem, null, null, null));
		// m_controller.button(Button.kX)
		// .whileTrue(CommandComposer.getFiveScoreRedAutoCommand(m_driveSubsystem, null,
		// null, null, null, null,
		// m_limeLightSubsystem, null, null, null));
		// m_controller.button(Button.kSquare)
		// .whileTrue(CommandComposer.getAlignToClosestAmpCommand(m_driveSubsystem,
		// m_limeLightSubsystem));
		// m_controller.button(Button.kCircle)
		// .whileTrue(CommandComposer.getMoveTowardClosestSpeakerCommand(1.5,
		// m_driveSubsystem,
		// m_limeLightSubsystem));
		// m_controller.button(Button.kTriangle)
		// .whileTrue(samples[7]);
	}

	public Command getAutonomousCommand() {
		return m_autoSelector.getSelected();
	}
}
