// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ControllerConstants.Axis;
import frc.robot.Constants.ControllerConstants.Button;
import frc.robot.Targeter.RegressionTargeter;
import frc.robot.commands.aimshooter.AimHeightCommand;
import frc.robot.commands.aimshooter.AimHeightCommand.AimHeightOperation;
import frc.robot.commands.climber.ClimberDriveCommand;
import frc.robot.commands.climber.ClimberPresetCommand;
import frc.robot.commands.climber.ClimberPresetCommand.ClimberOperation;
import frc.robot.commands.drive.PolarDriveCommand;
import frc.robot.commands.flywheel.FlywheelCommand;
import frc.robot.commands.flywheel.FlywheelCommand.FlywheelOperation;
import frc.robot.commands.indexer.IndexerCommand;
import frc.robot.commands.indexer.IndexerShootCommand;
import frc.robot.commands.indexer.IndexerStopCommand;
import frc.robot.subsystems.AimerSubsystem;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.ArduinoSubsystem.StatusCode;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.SimpleVisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	private final CommandGenericHID m_driverController = new CommandGenericHID(
			ControllerConstants.kDriverControllerPort);
	private final CommandGenericHID m_operatorController = new CommandGenericHID(
			ControllerConstants.kOperatorControllerPort);
	private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private final ArduinoSubsystem m_arduinoSubsystem = new ArduinoSubsystem();
	private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
	private final PneumaticsSubsystem m_pneumaticsSubsystem = new PneumaticsSubsystem();
	private final AimerSubsystem m_aimerSubsystem = new AimerSubsystem();
	private final RegressionTargeter m_targeter = new RegressionTargeter();
	private final SendableChooser<Command> m_autoSelector = new SendableChooser<Command>();
	private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
	private final SimpleVisionSubsystem m_visionSubsystem = new SimpleVisionSubsystem();
	private final FlywheelSubsystem m_flywheelSubsystem = new FlywheelSubsystem();
	private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
	final LimeLightSubsystem m_limeLightSubsystem = new PoseEstimationSubsystem() {
		{
			addPoseSupplier("BotPose@Odometry", () -> m_driveSubsystem.getPose());
		}
	};

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		CommandComposer.setSubsystems(m_driveSubsystem, m_arduinoSubsystem, m_pneumaticsSubsystem, m_aimerSubsystem,
				m_targeter, m_indexerSubsystem, m_visionSubsystem, m_flywheelSubsystem, m_intakeSubsystem,
				m_limeLightSubsystem);
		// m_autoSelector.addOption("Test Steering",
		// SetSteeringCommand.getCalibrationCommand(m_driveSubsystem));
		// m_autoSelector.addOption("PID Turn 90 degrees", new
		// TurnToAngleCommand(m_driveSubsystem, 90.0, 0.5, true));
		// m_autoSelector.addOption("Bang Bang Drive 2 Meters",
		// new BangBangDriveDistanceCommand(m_driveSubsystem, 2, 0.01));
		// m_autoSelector.addOption("Polar Drive Two Meters", new
		// PolarDriveCommand(m_driveSubsystem, 2, 180));
		m_autoSelector.addOption("Shoot and Leave Auto", CommandComposer.getShootAndLeaveAuto());
		m_autoSelector.addOption("Middle Two Score", CommandComposer.getTwoScoreMiddleAuto());
		// m_autoSelector.addOption("Right Two Score Blue",
		// CommandComposer.getTwoScoreRightAutoBlue());
		// m_autoSelector.addOption("Right Two Score Red",
		// CommandComposer.getTwoScoreRightAutoRed());
		// m_autoSelector.addOption("Left Two Score Blue",
		// CommandComposer.getTwoScoreLeftAutoBlue());
		// m_autoSelector.addOption("Left Two Score Red",
		// CommandComposer.getTwoScoreLeftAutoRed());
		// m_autoSelector.addOption("Right Three Score Blue",
		// CommandComposer.getThreeScoreRightAutoBlue());
		// m_autoSelector.addOption("Right Three Score Red",
		// CommandComposer.getThreeScoreRightAutoRed());
		// m_autoSelector.addOption("Left Three Score Blue",
		// CommandComposer.getThreeScoreLeftAutoBlue());
		// m_autoSelector.addOption("Left Three Score Red",
		// CommandComposer.getThreeScoreLeftAutoRed());
		// m_autoSelector.addOption("Right Four Score Blue",
		// CommandComposer.getFourScoreRightAutoBlue());
		// m_autoSelector.addOption("Right Four Score Red",
		// CommandComposer.getFourScoreRightAutoRed());
		// m_autoSelector.addOption("Left Four Score Blue",
		// CommandComposer.getFourScoreLeftAutoBlue());
		// m_autoSelector.addOption("Left Four Score Red",
		// CommandComposer.getFourScoreLeftAutoRed());
		// m_autoSelector.addOption("Five Score RED Auto (Start on Left)",
		// CommandComposer.getFiveScoreRedAutoCommand());
		// m_autoSelector.addOption("Five Score BLUE Auto (Start on Right)",
		// CommandComposer.getFiveScoreBlueAutoCommand());
		m_autoSelector.addOption("Four Score @red 3, middle 5, and middle 4 RED Auto (Start on Left)",
				CommandComposer.getTwoMiddleFourScoreRedCommand());
		m_autoSelector.addOption("Four Score @blue 3, middle 5, and middle 4 BLUE Auto (Start on Right)",
				CommandComposer.getTwoMiddleFourScoreBlueCommand());
		m_autoSelector.addOption("Four Score @middle 3, middle 4, and middle 5 RED Auto (Start on Left)",
				CommandComposer.getThreeMiddleFourScoreRedCommand());
		m_autoSelector.addOption("Four Score @middle 3, middle 4, and middle 5 BLUE Auto (Start on Right)",
				CommandComposer.getThreeMiddleFourScoreBlueCommand());
		m_autoSelector.addOption("Three Score @middle 4 and middle 5 BLUE Auto (Start on Right)",
				CommandComposer.getThreeScoreTwoMiddleBottomBlueAuto());
		m_autoSelector.addOption("Three Score @middle 4 and middle 5 RED Auto (Start on Right)",
				CommandComposer.getThreeScoreTwoMiddleBottomRedAuto());
		m_autoSelector.addOption("Four Score @blue1, middle 1, and middle 2 BLUE Auto (Start on Left)",
				CommandComposer.getFourScoreTwoMiddleTopBlueAuto());
		m_autoSelector.addOption("Four Score @red 1, middle 1, and middle 2 RED Auto (Start on Right)",
				CommandComposer.getFourScoreTwoMiddleTopRedAuto());
		m_autoSelector.addOption("Just leave", new PolarDriveCommand(m_driveSubsystem, 3, 180));
		// m_autoSelector.addOption("Get Blocks Auto", CommandComposer.getBlocksAuto());
		// m_autoSelector.addOption("Get Amp Auto Red",
		// CommandComposer.getAmpTwoAutoRed());
		// m_autoSelector.addOption("Get Amp Auto Blue",
		// CommandComposer.getAmpTwoAutoBlue());
		// m_autoSelector.addOption("Get Shoot and Amp Auto",
		// CommandComposer.getShootAndAmp());
		// m_autoSelector.addOption("Absolute to Zero", new
		// TurnToAngleCommand(m_driveSubsystem, 0, 0.5, false));
		// m_autoSelector.addOption("Intake With Sensor",
		// CommandComposer.getIntakeWithSensorCommand());
		// m_autoSelector.addOption("Intake With Sensor and Pneumatics",
		// CommandComposer.getTeleopIntakeCommand());
		// m_autoSelector.addOption("Ninja Star Command", new
		// NinjaStarCommand(m_driveSubsystem, 2, -2, 45));
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

		// --------------------- LED Controls ---------------------------
		// Should have RainbowPartyFunTime in the last 20 seconds of a match
		new Trigger(() -> DriverStation.getMatchTime() <= 20)
				.onTrue(m_arduinoSubsystem.writeStatus(StatusCode.RAINBOW_PARTY_FUN_TIME));
		// LEDs for when you want AMP
		m_operatorController.povLeft().onTrue(m_arduinoSubsystem.writeStatus(StatusCode.BLINKING_PURPLE));
		// LEDs for when you want CO-OP
		m_operatorController.povUp().onTrue(m_arduinoSubsystem.writeStatus(StatusCode.BLINKING_YELLOW));
		// LEDs for when you want HP to drop a note
		m_operatorController.povRight().onTrue(m_arduinoSubsystem.writeStatus(StatusCode.BLINKING_RED));
		// DEFAULT Button
		m_operatorController.povDown().onTrue(m_arduinoSubsystem.writeStatus(StatusCode.DEFAULT));
		// RainbowPartyFunTime
		m_operatorController.button(Button.kShare)
				.onTrue(m_arduinoSubsystem.writeStatus(StatusCode.RAINBOW_PARTY_FUN_TIME));

		// --------------------Drive Controls---------------------------------
		// D - Drive
		m_driveSubsystem.setDefaultCommand(m_driveSubsystem.driveCommand(
				() -> m_driverController.getRawAxis(Axis.kLeftY),
				() -> m_driverController.getRawAxis(Axis.kLeftX),
				() -> m_driverController.getRawAxis(Axis.kRightTrigger),
				() -> m_driverController.getRawAxis(Axis.kLeftTrigger)));
		m_driverController.button(Button.kOptions).onTrue(m_driveSubsystem.resetHeadingCommand());
		// D RIGHT BUMPER - Aim and Shoot
		m_driverController.button(Button.kRightBumper).whileTrue(CommandComposer.getAimAndShootCommand())
				.onFalse(new AimHeightCommand(m_aimerSubsystem, m_targeter, AimHeightOperation.SET_LOW)
						.andThen(new AimHeightCommand(m_aimerSubsystem, m_targeter, AimHeightOperation.SETTLE))
						.alongWith(m_flywheelSubsystem.stopFlywheel())
						.alongWith(new IndexerStopCommand(m_indexerSubsystem)));
		m_driverController.button(Button.kSquare)
				.whileTrue(m_driveSubsystem.robotOrientedDriveCommand(() -> m_driverController.getRawAxis(Axis.kLeftY),
						() -> m_driverController.getRawAxis(Axis.kLeftX),
						() -> m_driverController.getRawAxis(Axis.kRightTrigger),
						() -> m_driverController.getRawAxis(Axis.kLeftTrigger)));

		// -------------------Indexer Controls---------------------------------
		// D CIRCLE - Indexer shoot + stop flywheel
		m_driverController.button(Button.kCircle).onTrue(new IndexerShootCommand(m_indexerSubsystem)
				.andThen(m_flywheelSubsystem.stopFlywheel()));
		// OP RIGHT BUMPER - Indexer shoot + stop flywheel
		m_operatorController.button(Button.kRightBumper).onTrue(new IndexerShootCommand(m_indexerSubsystem)
				.andThen(m_flywheelSubsystem.stopFlywheel())
				.andThen(m_arduinoSubsystem.writeStatus(StatusCode.DEFAULT)));

		// ------------------Intake Controls-----------------------------------
		// OP break pad
		m_operatorController.button(Button.kTrackpad).onTrue(IndexerCommand.getFowardCommand(m_indexerSubsystem));
		// OP RIGHT TRIGGER - Up intake, reverse intake, wait, stop intake
		m_operatorController.button(Button.kRightTrigger)
				.onTrue(m_pneumaticsSubsystem.upIntakeCommand().andThen(m_intakeSubsystem.stopIntakeCommand())
						.andThen(m_arduinoSubsystem.writeStatus(StatusCode.DEFAULT)));
		// OP LEFT TRIGGER - Teleop Intake
		m_operatorController.button(Button.kLeftTrigger)
				.onTrue(CommandComposer.getTeleopIntakeCommand());
		// OP LEFT BUMPER + LEFT TRIGGER - Down intake, forward intake, wait, up intake,
		// forward index, stop intake, and then stop index
		m_operatorController.button(Button.kLeftBumper)
				.and(m_operatorController.button(Button.kLeftTrigger))
				.whileTrue(m_pneumaticsSubsystem.downIntakeCommand().andThen(m_intakeSubsystem.forwardIntakeCommand()))
				.onFalse(m_intakeSubsystem.forwardIntakeCommand()
						.andThen(new WaitCommand(0.5).alongWith(m_pneumaticsSubsystem.upIntakeCommand()).alongWith(
								IndexerCommand.getFowardCommand(m_indexerSubsystem).withTimeout(0.25)))
						.andThen(m_intakeSubsystem.stopIntakeCommand())
						.andThen(new IndexerStopCommand(m_indexerSubsystem)));
		// OP LEFT TRIGGER + LEFT BUMPER - Down Intake
		m_operatorController.button(Button.kLeftTrigger).and(m_operatorController.button(Button.kLeftBumper))
				.onTrue(m_pneumaticsSubsystem.downIntakeCommand());
		// OP LEFT BUMPER + POV UP - Stop Intake
		m_operatorController.povUp().and(m_operatorController.button(Button.kLeftBumper))
				.onTrue(m_intakeSubsystem.stopIntakeCommand());
		// sorry about this one
		// OP LEFT BUMPER + POV LEFT - Reverse Intake and Indexer w/ flywheel
		m_operatorController.povLeft().and(m_operatorController.button(Button.kLeftBumper)).onTrue(m_intakeSubsystem
				.reverseIntakeCommand().alongWith(IndexerCommand.getReverseCommand(m_indexerSubsystem))
				.alongWith(new FlywheelCommand(m_flywheelSubsystem, FlywheelOperation.SET_VELOCITY, -4000, -4000)));
		// OP LEFTBUMPER + POV RIGHT intake
		m_operatorController
				.povRight().and(m_operatorController.button(Button.kLeftBumper))
				.whileTrue(new FlywheelCommand(m_flywheelSubsystem, FlywheelOperation.SET_VELOCITY, -2000, -2000)
						.andThen(new AimHeightCommand(m_aimerSubsystem, m_targeter, AimHeightOperation.SOURCE)))
				.onFalse(sequence(
						IndexerCommand.getReverseCommand(m_indexerSubsystem),
						new WaitCommand(0.2),
						new IndexerStopCommand(m_indexerSubsystem), m_flywheelSubsystem.stopFlywheel()));

		m_operatorController.button(Button.kPS).onTrue(
				CommandComposer.getIntakeWithSensorCommand());

		// -------------------Climber Commands---------------------------------
		m_climberSubsystem.setDefaultCommand(new ClimberDriveCommand(m_climberSubsystem,
				() -> -m_operatorController.getRawAxis(Axis.kLeftY),
				() -> -m_operatorController.getRawAxis(Axis.kRightY)));
		m_driverController.button(Button.kTriangle)
				.onTrue(new ClimberPresetCommand(m_climberSubsystem, ClimberOperation.TOP,
						() -> m_operatorController.getRawAxis(Axis.kLeftY),
						() -> m_operatorController.getRawAxis(Axis.kRightY)));
		m_driverController.button(Button.kX)
				.onTrue(new ClimberPresetCommand(m_climberSubsystem, ClimberOperation.ZERO,
						() -> m_operatorController.getRawAxis(Axis.kLeftY),
						() -> m_operatorController.getRawAxis(Axis.kRightY)));

		// -------------------Aimer Commands-----------------------------------

		// OP CIRCLE - Default Aim
		m_operatorController.button(Button.kCircle).onTrue(
				new AimHeightCommand(m_aimerSubsystem, m_targeter, AimHeightOperation.SET_PRESET_DEFAULT)
						.andThen(new AimHeightCommand(m_aimerSubsystem, m_targeter, AimHeightOperation.SETTLE)));
		// OP SQUARE - Subwoofer Preset + everything
		m_operatorController.button(Button.kSquare).onTrue(
				new AimHeightCommand(m_aimerSubsystem, m_targeter, AimHeightOperation.PRESET_SUBWOOFER)
						.andThen(new AimHeightCommand(m_aimerSubsystem, m_targeter, AimHeightOperation.SETTLE)
								.alongWith(new FlywheelCommand(m_flywheelSubsystem, FlywheelOperation.SET_VELOCITY,
										8000, 8000))));
		// OP X - Amp Preset
		m_operatorController.button(Button.kX).onTrue(
				new AimHeightCommand(m_aimerSubsystem, m_targeter, AimHeightOperation.PRESET_AMP)
						.andThen(
								new AimHeightCommand(m_aimerSubsystem, m_targeter, AimHeightOperation.SETTLE).alongWith(
										new FlywheelCommand(m_flywheelSubsystem, FlywheelOperation.SET_VELOCITY, 500,
												2000)))); // 1800 old, 2000 new
		// OP TRIANGLE - Stop Flywheel
		m_operatorController.button(Button.kTriangle).onTrue(m_flywheelSubsystem.stopFlywheel());

		// ------------------Amp Bar Controls -------------------
		// m_operatorController.button(Button.kX)
		// .onTrue(CommandComposer.getAmpCommand(m_aimerSubsystem, m_targeter,
		// m_flywheelSubsystem));

		// TESTING TESTING TESTING
		// m_aimerSubsystem.setDefaultCommand(
		// new AimerDriveCommand(m_aimerSubsystem, () ->
		// m_driverController.getRawAxis(Axis.kRightY)));
		// m_operatorController.button(Button.kX).onTrue(m_pneumaticsSubsystem.toggleAmpBarCommand());

		// m_operatorController.button(Button.kShare).onTrue(CommandComposer.getBallPathTest(
		// m_intakeSubsystem, m_indexerSubsystem, m_flywheelSubsystem));
		// m_operatorController.button(Button.kPS).onTrue(CommandComposer.getIntakeWithSensorCommand(
		// ));
		// ------------------Intake Bar Controls, removal later-------------------
		// m_operatorController.button(Button.kTrackpad).onTrue(m_pneumaticsSubsystem.toggleIntakeCommand());
		// -------------------Flywheel Controls, removal later-----------------
		// m_driverController.button(Button.kPS)
		// .onTrue(new FlywheelCommand(m_flywheelSubsystem,
		// FlywheelOperation.SET_VELOCITY, 8000, 8000)); // 8000
		// m_driverController.button(Button.kPS)
		// .onFalse(m_flywheelSubsystem.stopFlywheel());
		// m_operatorController.button(Button.kOptions).onTrue(m_flywheelSubsystem.stopFlywheel());
		// m_driverController.button(Button.kRightBumper)
		// .onTrue(new SimpleVisionAlignCommand(m_driveSubsystem, m_visionSubsystem));
	}

	public Command getAutonomousCommand() {
		return m_autoSelector.getSelected();
	}
}