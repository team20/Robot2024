// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.swervebot;

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
import frc.robot.CommandComposer;
import frc.robot.Robot;
import frc.robot.Targeter.RegressionTargeter;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.drive.DriveDistanceCommand;
import frc.robot.subsystems.AdvantageScopeUtil;
import frc.robot.subsystems.AimerSubsystem;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLightEmulationSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.LimeLightSubsystem.Pose;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.swervebot.Constants.ControllerConstants;
import frc.swervebot.Constants.ControllerConstants.Axis;
import frc.swervebot.Constants.ControllerConstants.Button;

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
	private final PneumaticsSubsystem m_pneumaticsSubsystem = new PneumaticsSubsystem();
	private final AimerSubsystem m_aimerSubsystem = new AimerSubsystem();
	private final RegressionTargeter m_targeter = new RegressionTargeter();
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

		CommandComposer.setSubsystems(m_driveSubsystem, null, m_pneumaticsSubsystem, m_aimerSubsystem,
				m_targeter, null, null, null, null,
				m_limeLightSubsystem);

		// Configure the button bindings
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
				new DriveCommand(m_driveSubsystem, 1.0, 0, 0, 0.1, 5)
						.andThen(new DriveCommand(m_driveSubsystem, 0, 1, 0, 0.1, 5))
						.andThen(new DriveCommand(m_driveSubsystem, 0, 0, 45, 0.1, 5))
						.andThen(new DriveCommand(m_driveSubsystem, -1, -1, -45, 0.1, 5)),
				new DriveDistanceCommand(m_driveSubsystem, 1, 0.1)
						.andThen(new DriveDistanceCommand(m_driveSubsystem, -1, 0.1)),
		};
		m_controller.button(Button.kSquare)
				.whileTrue(CommandComposer.getDriveWhileAimingCommand(() -> m_controller.getRawAxis(Axis.kLeftY),
						() -> m_controller.getRawAxis(Axis.kLeftX), 5));
		// m_controller.button(Button.kSquare)
		// .whileTrue(CommandComposer.getFiveScoreBlue321C1());
		m_controller.button(Button.kX)
				.whileTrue(CommandComposer.getFiveScoreRed321C1());
		m_controller.button(Button.kCircle)
				.whileTrue(CommandComposer.getThreeScoreBlueC4C5());
		m_controller.button(Button.kTriangle)
				.whileTrue(
						CommandComposer.getThreeScoreRedC4C5());
		// m_controller.button(Button.kCircle)
		// .whileTrue(CommandComposer.getThreeScoreTwoMiddleBottomBlueAuto());
		// m_controller.button(Button.kTriangle)
		// .whileTrue(
		// CommandComposer.getThreeScoreTwoMiddleBottomRedAuto());
	}

	public Command getAutonomousCommand() {
		return m_autoSelector.getSelected();
	}
}
