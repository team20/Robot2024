package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.PoseConstants.*;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Targeter.RegressionTargeter;
import frc.robot.commands.TimedLEDCommand;
import frc.robot.commands.aimshooter.AimHeightCommand;
import frc.robot.commands.aimshooter.AimHeightCommand.AimHeightOperation;
import frc.robot.commands.drive.BangBangDriveDistanceCommand;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.drive.DriveDistanceCommand;
import frc.robot.commands.drive.PolarDriveCommand;
import frc.robot.commands.drive.TurnToAngleCommand;
import frc.robot.commands.flywheel.FlywheelCommand;
import frc.robot.commands.flywheel.FlywheelCommand.FlywheelOperation;
import frc.robot.commands.indexer.IndexWithSensorCommand;
import frc.robot.commands.indexer.IndexerCommand;
import frc.robot.commands.indexer.IndexerShootCommand;
import frc.robot.commands.indexer.IndexerStopCommand;
import frc.robot.subsystems.AimerSubsystem;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.ArduinoSubsystem.StatusCode;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.LimeLightSubsystem.Pose;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.SimpleVisionSubsystem;

// TODO UPDATE DOCUMENTATION
// TODO All the autos (update so it is not using the AimAndShoot teleop one)

public class CommandComposer {
	private static DriveSubsystem m_driveSubsystem;
	private static ArduinoSubsystem m_arduinoSubsystem;
	private static PneumaticsSubsystem m_pneumaticsSubsystem;
	private static AimerSubsystem m_aimerSubsystem;
	private static RegressionTargeter m_targeter;
	private static IndexerSubsystem m_indexerSubsystem;
	private static SimpleVisionSubsystem m_simpleVisionSubsystem;
	private static FlywheelSubsystem m_flywheelSubsystem;
	private static IntakeSubsystem m_intakeSubsystem;
	private static LimeLightSubsystem m_limeLightSubsystem;

	public static void setSubsystems(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, AimerSubsystem aimerSubsystem, RegressionTargeter targeter,
			IndexerSubsystem indexerSubsystem, SimpleVisionSubsystem simpleVisionSubsystem,
			FlywheelSubsystem flywheelSubsystem, IntakeSubsystem intakeSubsystem,
			LimeLightSubsystem limeLightSubsystem) {
		m_driveSubsystem = driveSubsystem;
		m_arduinoSubsystem = arduinoSubsystem;
		m_pneumaticsSubsystem = pneumaticsSubsystem;
		m_aimerSubsystem = aimerSubsystem;
		m_targeter = targeter;
		m_indexerSubsystem = indexerSubsystem;
		m_simpleVisionSubsystem = simpleVisionSubsystem;
		m_flywheelSubsystem = flywheelSubsystem;
		m_intakeSubsystem = intakeSubsystem;
		m_limeLightSubsystem = limeLightSubsystem;
	}

	/**
	 * Returns a command to shoot a note and leave the wing (end with LEDS).
	 * 
	 * @return The command.
	 */
	public static Command getShootAndLeaveAuto() {
		return sequence(
				getAimAndShootAuto(),
				new BangBangDriveDistanceCommand(m_driveSubsystem, -2, 0.01),
				new TimedLEDCommand(m_arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	public static Command getBlueShootAndLeaveAuto() {
		return sequence(
				getAimAndShootAuto(),
				DriveCommand.alignTo(new Pose(-6.0 + 0.55, 0.0 - 3.0, 180 - 25), 0.1, 5, m_driveSubsystem,
						m_limeLightSubsystem),
				new TimedLEDCommand(m_arduinoSubsystem, 0.25,
						StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	public static Command getRedShootAndLeaveAuto() {
		return sequence(
				getAimAndShootAuto(),
				DriveCommand.alignTo(new Pose(6.0 - 0.55, 0.0 - 3.0, 25), 0.1, 5, m_driveSubsystem,
						m_limeLightSubsystem),
				new TimedLEDCommand(m_arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	/**
	 * Returns a command for a two-score autonomous routine at SPEAKER middle.
	 * Shoot a note, drive forward to the next note, and shoot
	 * the note (end with LEDs).
	 * 
	 * If m_visionSubsystem is used, auto will include command
	 * to align with AprilTag.
	 * 
	 * @return The command.
	 */
	public static Command getTwoScoreMiddleAuto() {
		// SequentialCommandGroup alignCommand = new SequentialCommandGroup(
		// new PolarDriveCommand(m_driveSubsystem, 1., 90, 0.01));
		// if (m_simpleVisionSubsystem != null) {
		// alignCommand.addCommands(new SimpleVisionAlignCommand(m_driveSubsystem,
		// m_simpleVisionSubsystem));
		// }
		return sequence(
				parallel(
						m_pneumaticsSubsystem.downIntakeCommand(),
						getAimAndShootAuto()),
				race(
						getIntakeWithSensorCommand(),
						new PolarDriveCommand(m_driveSubsystem, 1, 180, 0.07)),
				getAimAndShootAuto(),
				new TimedLEDCommand(m_arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	/**
	 * Returns a command for a two-score autonomous routine on SPEAKER right.
	 * Shoot a note, drive forward to the next note, intake,
	 * turn to align (absolute -35 degrees), and shoot the note. End with LEDs.
	 * 
	 * If m_visionSubsystem is used, auto will include command
	 * to align with AprilTag.
	 * 
	 * @return The command.
	 */
	public static Command getTwoScoreRightAutoBlue() {
		// SequentialCommandGroup alignCommand = new SequentialCommandGroup(
		// new TurnToAngleCommand(m_driveSubsystem, -35, 2, false));
		// if (m_visionSubsystem != null) {
		// alignCommand.addCommands(new SimpleVisionAlignCommand(m_driveSubsystem,
		// m_visionSubsystem));
		// }
		return sequence(
				getAimAndShootAuto(),
				new PolarDriveCommand(m_driveSubsystem, 0.75, 180, 0.01),
				parallel(
						new PolarDriveCommand(m_driveSubsystem, 1.2, 240, 0.01),
						m_pneumaticsSubsystem.downIntakeCommand()),
				new TurnToAngleCommand(m_driveSubsystem, 0, 2, false),
				getIntakeWithSensorCommand(),
				new PolarDriveCommand(m_driveSubsystem, 0.2, -180, 0.01),
				getAimAndShootAuto(),
				// alignCommand,
				new TimedLEDCommand(m_arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	/**
	 * Returns a command for a two-score autonomous routine on SPEAKER right.
	 * Shoot a note, drive forward to the next note, intake,
	 * turn to align (absolute -35 degrees), and shoot the note. End with LEDs.
	 * 
	 * If m_visionSubsystem is used, auto will include command
	 * to align with AprilTag.
	 * 
	 * @return The command.
	 */
	public static Command getTwoScoreRightAutoRed() {
		// SequentialCommandGroup alignCommand = new SequentialCommandGroup(
		// new TurnToAngleCommand(m_driveSubsystem, -35, 2, false));
		// if (m_visionSubsystem != null) {
		// alignCommand.addCommands(new SimpleVisionAlignCommand(m_driveSubsystem,
		// m_visionSubsystem));
		// }
		return sequence(
				getAimAndShootAuto(),
				parallel(
						new PolarDriveCommand(m_driveSubsystem, 1.4, 240, 0.01),
						m_pneumaticsSubsystem.downIntakeCommand()),
				new TurnToAngleCommand(m_driveSubsystem, 0, 2, false),
				getIntakeWithSensorCommand(),
				new PolarDriveCommand(m_driveSubsystem, 0.2, -180, 0.01),
				getAimAndShootAuto(),
				// alignCommand,
				new TimedLEDCommand(m_arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	/**
	 * Returns a command for a two-score autonomous routine on SPEAKER left.
	 * Shoot a note, drive forward to the next note, intake,
	 * turn to align (absolute 35 degrees), and shoot the note. End with LEDs.
	 * 
	 * If m_visionSubsystem is used, auto will include command
	 * to align with AprilTag.
	 * 
	 * @return The command.
	 */
	public static Command getTwoScoreLeftAutoBlue() {
		// SequentialCommandGroup alignCommand = new SequentialCommandGroup(
		// new TurnToAngleCommand(m_driveSubsystem, 25, 2, false));
		// if (m_visionSubsystem != null) {
		// alignCommand.addCommands(new SimpleVisionAlignCommand(m_driveSubsystem,
		// m_visionSubsystem));
		// }
		return sequence(
				getSubwooferShotAuto(),
				parallel(
						new PolarDriveCommand(m_driveSubsystem, 1.2, -240, 0.01),
						m_pneumaticsSubsystem.downIntakeCommand()),
				new TurnToAngleCommand(m_driveSubsystem, 0, 2, false),
				getIntakeWithSensorCommand(),
				new PolarDriveCommand(m_driveSubsystem, 0.4, 180, 0.01),
				getAimAndShootAuto(),
				// alignCommand,
				new TimedLEDCommand(m_arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	public static Command getSubwooferShotAuto() {
		return sequence(
				parallel(
						new FlywheelCommand(m_flywheelSubsystem, FlywheelOperation.SET_VELOCITY, 4000, 4000),
						sequence(new AimHeightCommand(m_aimerSubsystem, null, AimHeightOperation.PRESET_SUBWOOFER))),
				new IndexerShootCommand(m_indexerSubsystem),
				m_flywheelSubsystem.stopFlywheel());
	}

	/**
	 * Returns a command for a two-score autonomous routine on SPEAKER left.
	 * Shoot a note, drive forward to the next note, intake,
	 * turn to align (absolute 35 degrees), and shoot the note. End with LEDs.
	 * 
	 * If m_visionSubsystem is used, auto will include command
	 * to align with AprilTag.
	 * 
	 * @return The command.
	 */
	public static Command getTwoScoreLeftAutoRed() {
		return sequence(
				getSubwooferShotAuto(),
				parallel(
						new PolarDriveCommand(m_driveSubsystem, 0.75, -180, 0.01),
						m_pneumaticsSubsystem.downIntakeCommand()),
				new PolarDriveCommand(m_driveSubsystem, 1, -240, 0.01),
				new TurnToAngleCommand(m_driveSubsystem, 0, 2, false),
				getIntakeWithSensorCommand(),
				new PolarDriveCommand(m_driveSubsystem, 0.2, 180, 0.01),
				getAimAndShootAuto(),
				new TimedLEDCommand(m_arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	/**
	 * Returns a command for a three-score autonomous routine on SPEAKER right.
	 * Use two-score routine: Shoot a note, drive forward to the next note,
	 * intake, turn to align, and shoot the note. Play LEDs.
	 * Third note: Turn to middle note, drive forward, intake,
	 * turn to align (absolute 12 degrees), and shoot the note. End with LEDS.
	 * 
	 * If m_visionSubsystem is used, auto will include command
	 * to align with AprilTag.
	 * 
	 * @return The command.
	 */
	public static Command getThreeScoreRightAutoBlue() {
		// SequentialCommandGroup alignCommand = new SequentialCommandGroup(
		// new TurnToAngleCommand(m_driveSubsystem, 0, 2, false));
		// if (m_visionSubsystem != null) {
		// alignCommand.addCommands(new SimpleVisionAlignCommand(m_driveSubsystem,
		// m_visionSubsystem));
		// }
		return sequence(
				// right note
				getTwoScoreRightAutoBlue(),
				// middle note
				new TurnToAngleCommand(m_driveSubsystem, 75, 2, false),
				new PolarDriveCommand(m_driveSubsystem, 1.25, 180, 0.01),
				getIntakeWithSensorCommand(),
				getAimAndShootAuto(),
				// alignCommand,
				new TimedLEDCommand(m_arduinoSubsystem, .25, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	/**
	 * Returns a command for a three-score autonomous routine on SPEAKER right.
	 * Use two-score routine: Shoot a note, drive forward to the next note,
	 * intake, turn to align, and shoot the note. Play LEDs.
	 * Third note: Turn to middle note, drive forward, intake,
	 * turn to align (absolute 12 degrees), and shoot the note. End with LEDS.
	 * 
	 * If m_visionSubsystem is used, auto will include command
	 * to align with AprilTag.
	 * 
	 * @return The command.
	 */
	public static Command getThreeScoreRightAutoRed() {
		return sequence(
				// right note
				getTwoScoreRightAutoRed(),
				// middle note
				new TurnToAngleCommand(m_driveSubsystem, 75, 2, false),
				new PolarDriveCommand(m_driveSubsystem, 1.25, 180, 0.01),
				getIntakeWithSensorCommand(),
				// alignCommand,
				getAimAndShootAuto(),
				new TimedLEDCommand(m_arduinoSubsystem, .25, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	/**
	 * Returns a command for a three-score autonomous routine on SPEAKER left.
	 * Use two-score routine: Shoot a note, drive forward to the next note,
	 * intake, turn to align, and shoot the note. Play LEDs.
	 * Third note: Turn to middle note, drive forward, intake,
	 * turn to align (absolute -12 degrees), and shoot the note. End with LEDS.
	 * 
	 * If m_visionSubsystem is used, auto will include command
	 * to align with AprilTag.
	 * 
	 * @return The command.
	 */
	public static Command getThreeScoreLeftAutoBlue() {
		return sequence(
				// right note
				getTwoScoreLeftAutoBlue(),
				// middle note
				new TurnToAngleCommand(m_driveSubsystem, -75, 2, false),
				new PolarDriveCommand(m_driveSubsystem, 1.25, -180, 0.01),
				getIntakeWithSensorCommand(),
				// alignCommand,
				getAimAndShootAuto(),
				new TimedLEDCommand(m_arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	/**
	 * Returns a command for a three-score autonomous routine on SPEAKER left.
	 * Use two-score routine: Shoot a note, drive forward to the next note,
	 * intake, turn to align, and shoot the note. Play LEDs.
	 * Third note: Turn to middle note, drive forward, intake,
	 * turn to align (absolute -12 degrees), and shoot the note. End with LEDS.
	 * 
	 * If m_visionSubsystem is used, auto will include command
	 * to align with AprilTag.
	 * 
	 * @return The command.
	 */
	public static Command getThreeScoreLeftAutoRed() {
		return sequence(
				// right note
				getTwoScoreLeftAutoRed(),
				// middle note
				new TurnToAngleCommand(m_driveSubsystem, -75, 2, false),
				new PolarDriveCommand(m_driveSubsystem, 1.25, -180, 0.01),
				getIntakeWithSensorCommand(),
				// alignCommand,
				getAimAndShootAuto(),
				new TimedLEDCommand(m_arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	/**
	 * Returns a command for a three-score autonomous routine on SPEAKER right.
	 * Use two-score routine: Shoot a note, drive forward to the next note,
	 * intake, turn to align, and shoot the note. Play LEDs.
	 * Third note: Turn to middle note, drive forward, intake,
	 * turn to align (absolute 12 degrees), and shoot the note. End with LEDS.
	 * 
	 * If m_visionSubsystem is used, auto will include command
	 * to align with AprilTag.
	 * 
	 * @return The command.
	 */
	public static Command getFourScoreRightAutoBlue() {
		// SequentialCommandGroup alignCommand = new SequentialCommandGroup(
		// new TurnToAngleCommand(m_driveSubsystem, 30, 2, false));
		// if (m_visionSubsystem != null) {
		// alignCommand.addCommands(new SimpleVisionAlignCommand(m_driveSubsystem,
		// m_visionSubsystem));
		// }
		return sequence(
				// right note and middle note
				getThreeScoreRightAutoBlue(),
				// left note
				new PolarDriveCommand(m_driveSubsystem, 2, 310, 0.01),
				new TurnToAngleCommand(m_driveSubsystem, 0, false),
				getIntakeWithSensorCommand(),
				new PolarDriveCommand(m_driveSubsystem, .5, 180, 0.01),
				getAimAndShootAuto(),
				// alignCommand,
				new TimedLEDCommand(m_arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	/**
	 * Returns a command for a three-score autonomous routine on SPEAKER right.
	 * Use two-score routine: Shoot a note, drive forward to the next note,
	 * intake, turn to align, and shoot the note. Play LEDs.
	 * Third note: Turn to middle note, drive forward, intake,
	 * turn to align (absolute 12 degrees), and shoot the note. End with LEDS.
	 * 
	 * If m_visionSubsystem is used, auto will include command
	 * to align with AprilTag.
	 * 
	 * @return The command.
	 */
	public static Command getFourScoreRightAutoRed() {
		// SequentialCommandGroup alignCommand = new SequentialCommandGroup(
		// new TurnToAngleCommand(m_driveSubsystem, 40, 2, false));
		// if (m_visionSubsystem != null) {
		// alignCommand.addCommands(new SimpleVisionAlignCommand(m_driveSubsystem,
		// m_visionSubsystem));
		// }
		return sequence(
				// right note and middle note
				getThreeScoreRightAutoRed(),
				// left note
				new PolarDriveCommand(m_driveSubsystem, 1.5, 90),
				new TurnToAngleCommand(m_driveSubsystem, 0, false),
				new PolarDriveCommand(m_driveSubsystem, .45, 0),
				parallel(
						new PolarDriveCommand(m_driveSubsystem, -0.2, 180), // -180
						m_intakeSubsystem.forwardIntakeCommand()),
				// alignCommand,
				getAimAndShootAuto(),
				new TimedLEDCommand(m_arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	/**
	 * Returns a command for a four-score autonomous routine on SPEAKER left.
	 * Use two-score routine: Shoot a note, drive forward to the next note,
	 * intake, turn to align, and shoot the note. Play LEDs.
	 * Third note: Turn to middle note, drive forward, intake,
	 * turn to align (absolute 12 degrees), and shoot the note. End with LEDS.
	 * 
	 * If m_visionSubsystem is used, auto will include command
	 * to align with AprilTag.
	 * 
	 * @return The command.
	 */
	public static Command getFourScoreLeftAutoBlue() {
		// SequentialCommandGroup alignCommand = new SequentialCommandGroup(
		// new TurnToAngleCommand(m_driveSubsystem, -35, 2, false));
		// if (m_visionSubsystem != null) {
		// alignCommand.addCommands(new SimpleVisionAlignCommand(m_driveSubsystem,
		// m_visionSubsystem));
		// }
		return sequence(
				// left note and middle note
				getThreeScoreLeftAutoBlue(),
				// right note
				new PolarDriveCommand(m_driveSubsystem, 2, -310, 0.01),
				new TurnToAngleCommand(m_driveSubsystem, 0, false),
				new PolarDriveCommand(m_driveSubsystem, .45, -180, 0.01),
				parallel(
						new PolarDriveCommand(m_driveSubsystem, -0.2, 180), // -180
						m_intakeSubsystem.forwardIntakeCommand()),
				getAimAndShootAuto(),
				// alignCommand,
				new TimedLEDCommand(m_arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	/**
	 * Returns a command for a four-score autonomous routine on SPEAKER left.
	 * Use two-score routine: Shoot a note, drive forward to the next note,
	 * intake, turn to align, and shoot the note. Play LEDs.
	 * Third note: Turn to middle note, drive forward, intake,
	 * turn to align (absolute 12 degrees), and shoot the note. End with LEDS.
	 * 
	 * If m_visionSubsystem is used, auto will include command
	 * to align with AprilTag.
	 * 
	 * @return The command.
	 */
	public static Command getFourScoreLeftAutoRed() {
		// SequentialCommandGroup alignCommand = new SequentialCommandGroup(
		// new TurnToAngleCommand(m_driveSubsystem, -20, 2, false));
		// if (m_visionSubsystem != null) {
		// alignCommand.addCommands(new SimpleVisionAlignCommand(m_driveSubsystem,
		// m_visionSubsystem));
		// }
		return sequence(
				// left note and middle note
				getThreeScoreLeftAutoRed(),
				// right note
				new PolarDriveCommand(m_driveSubsystem, 1.8, -280, 0.01),
				new TurnToAngleCommand(m_driveSubsystem, 0, false),
				new PolarDriveCommand(m_driveSubsystem, .35, 180, 0.01), // -180
				m_intakeSubsystem.forwardIntakeCommand(),
				getAimAndShootAuto(),
				// alignCommand,
				new TimedLEDCommand(m_arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	/**
	 * 
	 * 
	 * @return The command.
	 */
	public static Command getAmpTwoAutoBlue() {
		return sequence(
				// strafe towards speaker, shoot
				m_pneumaticsSubsystem.downIntakeCommand(),
				new PolarDriveCommand(m_driveSubsystem, 0.22, -90),
				new PolarDriveCommand(m_driveSubsystem, 0.4, 0),
				getAmpCommand(),
				new IndexerShootCommand(m_indexerSubsystem),
				m_flywheelSubsystem.stopFlywheel(),
				new TimedLEDCommand(m_arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME));
		// strafe away, intake
		// new PolarDriveCommand(m_driveSubsystem, 1.0, 0.2, -90, 5.0));
		// new PolarDriveCommand(m_driveSubsystem, 0.4, 180),
		// getIntakeWithSensorCommand(m_intakeSubsystem, m_indexerSubsystem,
		// m_arduinoSubsystem),
		// // go back and shoot
		// new PolarDriveCommand(m_driveSubsystem, 1, 90),
		// new PolarDriveCommand(m_driveSubsystem, 0.4, 0),
		// getAmpCommand(),
		// new IndexerShootCommand(m_indexerSubsystem),
		// m_flywheelSubsystem.stopFlywheel(),
		// new TimedLEDCommand(m_arduinoSubsystem, 0.4,
		// StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	/**
	 * 
	 * 
	 * @return The command.
	 */
	public static Command getAmpTwoAutoRed() {
		return sequence(
				// strafe towards speaker, shoot
				new PolarDriveCommand(m_driveSubsystem, 0.22, 90),
				new PolarDriveCommand(m_driveSubsystem, 0.4, 0),
				getAmpCommand(),
				new IndexerShootCommand(m_indexerSubsystem),
				m_flywheelSubsystem.stopFlywheel(),
				new TimedLEDCommand(m_arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME),
				// strafe away, intake
				new PolarDriveCommand(m_driveSubsystem, 1, 90),
				new PolarDriveCommand(m_driveSubsystem, 0.4, 180),
				getIntakeWithSensorCommand(),
				// go back and shoot
				new PolarDriveCommand(m_driveSubsystem, 1, -90),
				new PolarDriveCommand(m_driveSubsystem, 0.4, 0),
				getAmpCommand(),
				new IndexerShootCommand(m_indexerSubsystem),
				m_flywheelSubsystem.stopFlywheel(),
				new TimedLEDCommand(m_arduinoSubsystem, 0.4, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	/**
	 * Returns a command for
	 * 
	 * @return The command.
	 */
	public static Command getShootAndAmp() {
		return sequence(
				getAimAndShootAuto(),
				// strafe towards speaker, shoot
				new PolarDriveCommand(m_driveSubsystem, 1.4, -240, 0.01),
				new TurnToAngleCommand(m_driveSubsystem, 0, 2, false),
				new PolarDriveCommand(m_driveSubsystem, 0.4, 180, 0.01),
				getIntakeWithSensorCommand(),
				// strafe away, intake
				new TurnToAngleCommand(m_driveSubsystem, -90, 2, false),
				new PolarDriveCommand(m_driveSubsystem, 0.70, 90),
				new PolarDriveCommand(m_driveSubsystem, -1.05, 180),
				getAmpCommand(),
				new IndexerShootCommand(m_indexerSubsystem),
				m_flywheelSubsystem.stopFlywheel());
	}

	/**
	 * Returns a command to drive back and forth various amounts.
	 * 
	 * @return The command.
	 */
	public static Command getBlocksAuto() {
		return sequence(
				DriveDistanceCommand.create(m_driveSubsystem, 0.75),
				DriveDistanceCommand.create(m_driveSubsystem, -1.25),
				DriveDistanceCommand.create(m_driveSubsystem, 1.5),
				DriveDistanceCommand.create(m_driveSubsystem, -1.825),
				DriveDistanceCommand.create(m_driveSubsystem, 2.125),
				DriveDistanceCommand.create(m_driveSubsystem, -2.5),
				new TimedLEDCommand(m_arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	public static Command getIntakeWithSensorCommand() {
		return sequence(
				parallel(
						new IndexWithSensorCommand(m_indexerSubsystem, 0.5),
						m_intakeSubsystem.forwardIntakeCommand(),
						m_arduinoSubsystem.writeStatus(StatusCode.SOLID_ORANGE)),
				m_intakeSubsystem.stopIntakeCommand(),
				m_arduinoSubsystem.writeStatus(StatusCode.DEFAULT));
	}

	public static Command getAmpCommand() {
		return sequence(
				new AimHeightCommand(m_aimerSubsystem, m_targeter, AimHeightOperation.SET_PRESET_DEFAULT),
				new FlywheelCommand(m_flywheelSubsystem, FlywheelOperation.SET_VELOCITY, 1500, 1500)); // 1300 1650

	}

	public static Command getTeleopIntakeCommand() {
		return sequence(
				m_pneumaticsSubsystem.downIntakeCommand(),
				getIntakeWithSensorCommand(),
				m_pneumaticsSubsystem.upIntakeCommand());
	}

	public static Command getBallPathTest() {
		return parallel(
				new FlywheelCommand(m_flywheelSubsystem, FlywheelOperation.SET_VELOCITY, 8000, 8000),
				m_intakeSubsystem.forwardIntakeCommand(),
				IndexerCommand.getFowardCommand(m_indexerSubsystem));
	}

	public static Command getAimAndShootCommand() {
		return sequence(
				new AimHeightCommand(m_aimerSubsystem, m_targeter, AimHeightOperation.SET_PRESET_DEFAULT),
				parallel(
						sequence(
								new FlywheelCommand(m_flywheelSubsystem, FlywheelOperation.SET_VELOCITY, 8000, 8000),
								new FlywheelCommand(m_flywheelSubsystem, FlywheelOperation.SETTLE, 0, 0)),
						new AimHeightCommand(m_aimerSubsystem, m_targeter,
								AimHeightOperation.CALC_AND_SET, m_limeLightSubsystem)
										.andThen(m_arduinoSubsystem.writeStatus(StatusCode.SOLID_BLUE)),
						// new SimpleVisionAlignCommand(m_driveSubsystem, m_simpleVisionSubsystem)),
						getTurnToClosestSpeakerCommand()));
		// new IndexerShootCommand(m_indexerSubsystem),
		// m_flywheelSubsystem.stopFlywheel());
	}

	public static Command getAimAndShootAuto() {
		return sequence(
				getAimAndShootCommand()
						.withTimeout(2),
				new IndexerShootCommand(m_indexerSubsystem),
				m_flywheelSubsystem.stopFlywheel());
	}

	public static Command getSourcePickUpCommand() {
		return parallel(
				new AimHeightCommand(m_aimerSubsystem, m_targeter, AimHeightOperation.SOURCE),
				new FlywheelCommand(m_flywheelSubsystem, FlywheelOperation.SET_VELOCITY, -2000, -2000));
		// IndexerCommand.getReverseCommand(m_indexerSubsystem),
		// m_intakeSubsystem.reverseIntakeCommand()

	}

	public static Command getStopFlywheelAndIndexer() {
		return parallel(
				new IndexerStopCommand(m_indexerSubsystem),
				m_intakeSubsystem.stopIntakeCommand(),
				m_flywheelSubsystem.stopFlywheel());
	}

	public static Command getTurnToClosestSpeakerCommand() {
		Supplier<Pose2d> s = () -> {
			var target = m_limeLightSubsystem.closest(kBlueSpeakerPosition, kRedSpeakerPosition);
			var t = m_limeLightSubsystem.transformationToward(target);
			return m_driveSubsystem.getPose().plus(t);
		};
		return new DriveCommand(m_driveSubsystem, s, 0.1, 5);
	}

	public static Command getMoveTowardClosestSpeakerCommand(double distanceToTarget) {
		Supplier<Pose2d> s = () -> {
			var target = m_limeLightSubsystem.closest(kBlueSpeakerPosition, kRedSpeakerPosition);
			var t = m_limeLightSubsystem.transformationToward(target, distanceToTarget);
			return m_driveSubsystem.getPose().plus(t);
		};
		return new DriveCommand(m_driveSubsystem, s, 0.1, 5);
	}

	public static Command getAlignToClosestAmpCommand() {
		Supplier<Pose2d> s1 = () -> {
			Pose target = new Pose(m_limeLightSubsystem.closest(kBlueAmpPose, kRedAmpPose));
			var t = m_limeLightSubsystem.transformationTo(target.add(new Pose(0, -0.3, 0)));
			return m_driveSubsystem.getPose().plus(t);
		};
		Supplier<Pose2d> s2 = () -> {
			var target = m_limeLightSubsystem.closest(kBlueAmpPose, kRedAmpPose);
			var t = m_limeLightSubsystem.transformationTo(target);
			return m_driveSubsystem.getPose().plus(t);
		};
		return new DriveCommand(m_driveSubsystem, s1, 0.1, 5).andThen(new DriveCommand(m_driveSubsystem, s2, 0.1, 5));
	}

	public static Command getShootToClosestSpeakerAtCommand(Pose2d targetPose, double timeout) {
		return DriveCommand.alignTo(targetPose, 0.1, 5, m_driveSubsystem, m_limeLightSubsystem).withTimeout(timeout)
				.andThen(getAimAndShootAuto());
	}

	public static Command getPickUpNoteAtCommand(Pose2d targetPose) {
		return DriveCommand
				// TODO: originally .3
				.alignTo(targetPose.plus(new Transform2d(0.6, 0, Rotation2d.fromDegrees(0))), 0.2, 5, m_driveSubsystem,
						m_limeLightSubsystem)
				.andThen(parallel(
						DriveCommand.alignTo(targetPose, 0.1, 5, m_driveSubsystem, m_limeLightSubsystem),
						getIntakeWithSensorCommand()));
	}

	public static Command getPickUpNoteAndShootAtCommand(Pose2d targetPose, double timeout) {
		return sequence(
				getPickUpNoteAtCommand(targetPose).withTimeout(timeout),
				getAimAndShootAuto());
	}

	public static Command getFourScoreBlueAuto() {
		return sequence(
				m_pneumaticsSubsystem.downIntakeCommand(),
				getShootToClosestSpeakerAtCommand(kBlueNoteThreePose.add(new Pose(-0.65, 0, 0)), 1.5),
				// 2nd note
				getPickUpNoteAndShootAtCommand(kBlueNoteThreePose, 2.5),
				// 3rd note
				getPickUpNoteAndShootAtCommand(kBlueNoteTwoPose, 2.5),
				// 4th note
				getPickUpNoteAndShootAtCommand(kBlueNoteOnePose, 2.5));
	}

	public static Command getFourScoreRedAuto() {
		return sequence(
				m_pneumaticsSubsystem.downIntakeCommand(),
				getShootToClosestSpeakerAtCommand(kRedNoteThreePose.add(new Pose(0.6, 0, 0)), 1.5),
				// 2nd note
				getPickUpNoteAndShootAtCommand(kRedNoteThreePose, 2.5),
				// 3rd note
				getPickUpNoteAndShootAtCommand(kRedNoteTwoPose, 2.5),
				// 4th note
				getPickUpNoteAndShootAtCommand(kRedNoteOnePose, 2.5));
	}

	public static Command getFiveScoreBlueAuto() {
		return sequence(
				getFourScoreBlueAuto(),
				getPickUpNoteAtCommand(kBlueCenterNoteOnePose).withTimeout(4),
				getShootToClosestSpeakerAtCommand(kBlueNoteOnePose, 4));
	}

	public static Command getFiveScoreRedAuto() {
		return sequence(getFourScoreRedAuto(),
				getPickUpNoteAtCommand(kRedCenterNoteOnePose).withTimeout(4),
				getShootToClosestSpeakerAtCommand(kRedNoteOnePose, 4));
	}

	public static Command getTwoMiddleFourScoreRedCommand() {
		return sequence(
				m_pneumaticsSubsystem.downIntakeCommand(),
				getAimAndShootAuto(),
				getPickUpNoteAndShootAtCommand(kRedNoteThreePose, 5),
				DriveCommand.moveToward(new Translation2d(6, -2), 0, 0.5, 360, m_driveSubsystem, m_limeLightSubsystem),
				getPickUpNoteAtCommand(kRedCenterNoteFivePose),
				DriveCommand.moveToward(new Translation2d(6, -2), 0, 0.5, 360, m_driveSubsystem, m_limeLightSubsystem),
				getShootToClosestSpeakerAtCommand(new Pose(6, -0.5, 25), 3),
				DriveCommand.moveToward(new Translation2d(6, -3), 0, 0.5, 360, m_driveSubsystem, m_limeLightSubsystem),
				DriveCommand.moveToward(new Translation2d(2, -3), 0, 0.5, 360, m_driveSubsystem, m_limeLightSubsystem),
				getPickUpNoteAtCommand(kRedCenterNoteFourPose),
				DriveCommand.moveToward(new Translation2d(6, -3), 0, 0.5, 360, m_driveSubsystem, m_limeLightSubsystem),
				getShootToClosestSpeakerAtCommand(new Pose(6, -0.5, 25), 3)

		);
	}

	public static Command getTwoMiddleFourScoreBlueCommand() {
		return sequence(
				m_pneumaticsSubsystem.downIntakeCommand(),
				getAimAndShootAuto(),
				getPickUpNoteAndShootAtCommand(kBlueNoteThreePose, 5),
				DriveCommand.moveToward(new Translation2d(-6, -2), 0, 0.5, 360, m_driveSubsystem, m_limeLightSubsystem),
				getPickUpNoteAtCommand(kBlueCenterNoteFivePose),
				DriveCommand.moveToward(new Translation2d(-6, -2), 0, 0.5, 360, m_driveSubsystem, m_limeLightSubsystem),
				getShootToClosestSpeakerAtCommand(new Pose(-6, -0.5, 155), 3),
				DriveCommand.moveToward(new Translation2d(-6, -3), 0, 0.5, 360, m_driveSubsystem, m_limeLightSubsystem),
				DriveCommand.moveToward(new Translation2d(-2, -3), 0, 0.5, 360, m_driveSubsystem, m_limeLightSubsystem),
				getPickUpNoteAtCommand(kBlueCenterNoteFourPose),
				DriveCommand.moveToward(new Translation2d(-6, -3), 0, 0.5, 360, m_driveSubsystem, m_limeLightSubsystem),
				getShootToClosestSpeakerAtCommand(new Pose(-6, -0.5, 155), 3)

		);
	}

	public static Command getThreeMiddleFourScoreBlueCommand() {
		return sequence(
				m_pneumaticsSubsystem.downIntakeCommand(),
				getAimAndShootAuto(),
				DriveCommand.moveToward(new Translation2d(-6, -2), 0, 0.5, 360, m_driveSubsystem, m_limeLightSubsystem),
				getPickUpNoteAtCommand(kBlueCenterNoteFivePose).withTimeout(4),
				DriveCommand.moveToward(new Translation2d(-6, -2), 0, 0.5, 360, m_driveSubsystem, m_limeLightSubsystem),
				getShootToClosestSpeakerAtCommand(new Pose(-6, -0.5, 155), 3),
				DriveCommand.moveToward(new Translation2d(-6, -3), 0, 0.5, 360, m_driveSubsystem, m_limeLightSubsystem),
				DriveCommand.moveToward(new Translation2d(-2, -3), 0, 0.5, 360, m_driveSubsystem, m_limeLightSubsystem),
				getPickUpNoteAtCommand(kBlueCenterNoteFourPose).withTimeout(2),
				DriveCommand.moveToward(new Translation2d(-6, -3), 0, 0.5, 360, m_driveSubsystem, m_limeLightSubsystem),
				DriveCommand.moveToward(new Translation2d(-1, -3), 0, 0.5, 360, m_driveSubsystem, m_limeLightSubsystem),
				getPickUpNoteAtCommand(kBlueCenterNoteThreePose).withTimeout(2),
				DriveCommand.moveToward(new Translation2d(-1, -3), 0, 0.5, 360, m_driveSubsystem, m_limeLightSubsystem),
				DriveCommand.moveToward(new Translation2d(-6, -3), 0, 0.5, 360, m_driveSubsystem, m_limeLightSubsystem),
				getShootToClosestSpeakerAtCommand(new Pose(-6, -0.5, 25), 3)

		);
	}

	public static Command getThreeMiddleFourScoreRedCommand() {
		return sequence(
				m_pneumaticsSubsystem.downIntakeCommand(),
				getAimAndShootAuto(),
				DriveCommand.moveToward(new Translation2d(6, -2), 0, 0.5, 360, m_driveSubsystem, m_limeLightSubsystem),
				getPickUpNoteAtCommand(kRedCenterNoteFivePose).withTimeout(4),
				DriveCommand.moveToward(new Translation2d(6, -2), 0, 0.5, 360, m_driveSubsystem, m_limeLightSubsystem),
				getShootToClosestSpeakerAtCommand(new Pose(6, -0.5, 25), 3),
				DriveCommand.moveToward(new Translation2d(6, -3), 0, 0.5, 360, m_driveSubsystem, m_limeLightSubsystem),
				DriveCommand.moveToward(new Translation2d(2, -3), 0, 0.5, 360, m_driveSubsystem, m_limeLightSubsystem),
				getPickUpNoteAtCommand(kRedCenterNoteFourPose).withTimeout(2),
				DriveCommand.moveToward(new Translation2d(6, -3), 0, 0.5, 360, m_driveSubsystem, m_limeLightSubsystem),
				getShootToClosestSpeakerAtCommand(new Pose(6, -0.5, 25), 3),
				DriveCommand.moveToward(new Translation2d(6, -3), 0, 0.5, 360, m_driveSubsystem, m_limeLightSubsystem),
				DriveCommand.moveToward(new Translation2d(1, -3), 0, 0.5, 360, m_driveSubsystem, m_limeLightSubsystem),
				getPickUpNoteAtCommand(kRedCenterNoteThreePose).withTimeout(2),
				DriveCommand.moveToward(new Translation2d(1, -3), 0, 0.5, 360, m_driveSubsystem, m_limeLightSubsystem),
				DriveCommand.moveToward(new Translation2d(6, -3), 0, 0.5, 360, m_driveSubsystem, m_limeLightSubsystem),
				getShootToClosestSpeakerAtCommand(new Pose(6, -0.5, 25), 3)

		);
	}

	// TODO: things may go very very wrong...
	public static Command getThreeScoreTwoMiddleBottomBlueAuto() {
		return sequence(
				getAimAndShootAuto(),
				DriveCommand.alignTo(new Pose(-5, -3, 180 + 25), 0.3, 10, m_driveSubsystem, m_limeLightSubsystem),
				m_pneumaticsSubsystem.downIntakeCommand(),
				getPickUpNoteAtCommand(kBlueCenterNoteFourPose).withTimeout(4),
				DriveCommand.alignTo(new Pose(-5.5, -2.3, 180 + 0), 0.3, 10, m_driveSubsystem, m_limeLightSubsystem),
				DriveCommand.alignTo(new Pose(-6.25, 0.4, 180 + 0), 0.3, 10, m_driveSubsystem, m_limeLightSubsystem),
				getAimAndShootAuto(),
				DriveCommand.alignTo(new Pose(-5.5, -3, 180 + 0), 0.3, 10, m_driveSubsystem, m_limeLightSubsystem),
				getPickUpNoteAtCommand(kBlueCenterNoteFivePose).withTimeout(4),
				DriveCommand.alignTo(new Pose(-5.5, -3, 180 + 0), 0.3, 10, m_driveSubsystem, m_limeLightSubsystem),
				DriveCommand.alignTo(new Pose(-6.25, 0.4, 180 + 0), 0.3, 10, m_driveSubsystem, m_limeLightSubsystem),
				getAimAndShootAuto());
	}

	// TODO: things may go very very wrong...
	public static Command getThreeScoreTwoMiddleBottomRedAuto() {
		return sequence(
				m_pneumaticsSubsystem.downIntakeCommand(),
				getAimAndShootAuto(),
				DriveCommand.alignTo(new Pose(5, -3, -25), 0.3, 10, m_driveSubsystem, m_limeLightSubsystem),
				m_pneumaticsSubsystem.downIntakeCommand(),
				getPickUpNoteAtCommand(kRedCenterNoteFourPose).withTimeout(4),
				DriveCommand.alignTo(new Pose(5.5, -2.3, 0), 0.3, 10, m_driveSubsystem, m_limeLightSubsystem),
				DriveCommand.alignTo(new Pose(6.25, 0.4, 0), 0.3, 10, m_driveSubsystem, m_limeLightSubsystem),
				getAimAndShootAuto(),
				DriveCommand.alignTo(new Pose(5.5, -3, 0), 0.3, 10, m_driveSubsystem, m_limeLightSubsystem),
				getPickUpNoteAtCommand(kRedCenterNoteFivePose).withTimeout(4),
				DriveCommand.alignTo(new Pose(5.5, -3, 0), 0.3, 10, m_driveSubsystem, m_limeLightSubsystem),
				DriveCommand.alignTo(new Pose(6.25, 0.4, 0), 0.3, 10, m_driveSubsystem, m_limeLightSubsystem),
				getAimAndShootAuto());
	}

	public static Command getThreeScoreOneMiddleTopBlueAuto() {
		return sequence(m_pneumaticsSubsystem.downIntakeCommand(),
				getShootToClosestSpeakerAtCommand(kBlueNoteOnePose.add(new Pose(-0.4, -0.4, 0)), 1.5),
				// 2nd note
				getPickUpNoteAndShootAtCommand(kBlueNoteOnePose, 1.5),
				// 3rd note
				getPickUpNoteAtCommand(kBlueCenterNoteOnePose).withTimeout(4),
				getShootToClosestSpeakerAtCommand(kBlueNoteOnePose, 4));
	}

	public static Command getThreeScoreOneMiddleTopRedAuto() {
		return sequence(m_pneumaticsSubsystem.downIntakeCommand(),
				getShootToClosestSpeakerAtCommand(kRedNoteOnePose.add(new Pose(0.4, -0.4, 0)), 1.5),
				// 2nd note
				getPickUpNoteAndShootAtCommand(kRedNoteOnePose, 1.5),
				// 3rd note
				getPickUpNoteAtCommand(kRedCenterNoteOnePose).withTimeout(4),
				getShootToClosestSpeakerAtCommand(kRedNoteOnePose, 4));
	}

	public static Command getFourScoreTwoMiddleTopBlueAuto() {
		return sequence(getThreeScoreOneMiddleTopBlueAuto(),
				// 4th note
				getPickUpNoteAtCommand(kBlueCenterNoteTwoPose).withTimeout(4),
				getShootToClosestSpeakerAtCommand(kBlueNoteOnePose, 4));
	}

	public static Command getFourScoreTwoMiddleTopRedAuto() {
		return sequence(getThreeScoreOneMiddleTopRedAuto(),
				// 4th note
				getPickUpNoteAtCommand(kRedCenterNoteTwoPose).withTimeout(4),
				getShootToClosestSpeakerAtCommand(kRedNoteOnePose, 4));
	}
}
