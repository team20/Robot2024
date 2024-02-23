package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SimpleVisionAlignCommand;
import frc.robot.commands.TimedLEDCommand;
import frc.robot.commands.drive.BangBangDriveDistanceCommand;
import frc.robot.commands.drive.DriveDistanceCommand;
import frc.robot.commands.drive.PolarDriveCommand;
import frc.robot.commands.drive.TurnToAngleCommand;
import frc.robot.commands.indexer.IndexWithSensorCommand;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.ArduinoSubsystem.StatusCode;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.SimpleVisionSubsystem;

public class CommandComposer {
	/**
	 * Returns a command to shoot a note and leave the wing (end with LEDS).
	 * 
	 * @param driveSubsystem   The drive subsystem.
	 * @param arduinoSubsystem The arduino subsystem for LEDs.
	 * @return The command.
	 */
	public static Command getShootAndLeaveAuto(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return sequence(
				new BangBangDriveDistanceCommand(driveSubsystem, 2, 0.01),
				new TimedLEDCommand(arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	/**
	 * Returns a command for a two-score autonomous routine at SPEAKER middle.
	 * Shoot a note, drive forward to the next note, and shoot
	 * the note (end with LEDs).
	 * 
	 * @param driveSubsystem   The drive subsystem.
	 * @param arduinoSubsystem The arduino subsystem for LEDs.
	 * @param visionSubsystem  The vision subsystem with Limelight.
	 *                         If visionSubsystem is used, auto will include command
	 *                         to align with AprilTag.
	 * @return The command.
	 */
	public static Command getTwoScoreMiddleAuto(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem,
			SimpleVisionSubsystem visionSubsystem) {
		SequentialCommandGroup alignCommand = new SequentialCommandGroup(
				new PolarDriveCommand(driveSubsystem, 1., 90, 0.01));
		if (visionSubsystem != null) {
			alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem));
		}
		return sequence(
				alignCommand,
				new TimedLEDCommand(arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	/**
	 * Returns a command for a two-score autonomous routine on SPEAKER right.
	 * Shoot a note, drive forward to the next note, intake,
	 * turn to align (absolute -35 degrees), and shoot the note. End with LEDs.
	 * 
	 * @param driveSubsystem   The drive subsystem.
	 * @param arduinoSubsystem The arduino subsystem for LEDs.
	 * @param visionSubsystem  The vision subsystem with Limelight.
	 *                         If visionSubsystem is used, auto will include command
	 *                         to align with AprilTag.
	 * @return The command.
	 */
	public static Command getTwoScoreRightAuto(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem,
			SimpleVisionSubsystem visionSubsystem) {
		SequentialCommandGroup alignCommand = new SequentialCommandGroup(
				new TurnToAngleCommand(driveSubsystem, -35, 2, false));
		if (visionSubsystem != null) {
			alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem));
		}
		return sequence(
				new PolarDriveCommand(driveSubsystem, 0.75, 180, 0.01),
				new PolarDriveCommand(driveSubsystem, 1.2, 240, 0.01),
				new TurnToAngleCommand(driveSubsystem, 0, 2, false),
				new PolarDriveCommand(driveSubsystem, 0.2, -180, 0.01),
				alignCommand,
				new TimedLEDCommand(arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	/**
	 * Returns a command for a two-score autonomous routine on SPEAKER left.
	 * Shoot a note, drive forward to the next note, intake,
	 * turn to align (absolute 35 degrees), and shoot the note. End with LEDs.
	 * 
	 * @param driveSubsystem   The drive subsystem.
	 * @param arduinoSubsystem The arduino subsystem for LEDs.
	 * @param visionSubsystem  The vision subsystem with Limelight.
	 *                         If visionSubsystem is used, auto will include command
	 *                         to align with AprilTag.
	 * 
	 * @return The command.
	 */
	public static Command getTwoScoreLeftAuto(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem,
			SimpleVisionSubsystem visionSubsystem) {
		SequentialCommandGroup alignCommand = new SequentialCommandGroup(
				new TurnToAngleCommand(driveSubsystem, 35, 2, false));
		if (visionSubsystem != null) {
			alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem));
		}
		return sequence(
				new PolarDriveCommand(driveSubsystem, 1.4, -240, 0.01),
				new TurnToAngleCommand(driveSubsystem, 0, 2, false),
				new PolarDriveCommand(driveSubsystem, 0.4, 180, 0.01),
				alignCommand,
				new TimedLEDCommand(arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	/**
	 * Returns a command for a three-score autonomous routine on SPEAKER right.
	 * Use two-score routine: Shoot a note, drive forward to the next note,
	 * intake, turn to align, and shoot the note. Play LEDs.
	 * Third note: Turn to middle note, drive forward, intake,
	 * turn to align (absolute 12 degrees), and shoot the note. End with LEDS.
	 * 
	 * @param driveSubsystem   The drive subsystem.
	 * @param arduinoSubsystem The arduino subsystem for LEDs.
	 * @param visionSubsystem  The vision subsystem with Limelight.
	 *                         If visionSubsystem is used, auto will include command
	 *                         to align with AprilTag.
	 * @return The command.
	 */
	public static Command getThreeScoreRightAuto(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem,
			SimpleVisionSubsystem visionSubsystem) {
		SequentialCommandGroup alignCommand = new SequentialCommandGroup(
				new TurnToAngleCommand(driveSubsystem, 0, 2, false));
		if (visionSubsystem != null) {
			alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem));
		}
		return sequence(
				// right note
				getTwoScoreRightAuto(driveSubsystem, arduinoSubsystem, visionSubsystem),
				// middle note
				new TurnToAngleCommand(driveSubsystem, 75, 2, false),
				new PolarDriveCommand(driveSubsystem, 1.25, 180, 0.01),
				alignCommand,
				new TimedLEDCommand(arduinoSubsystem, .25, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	/**
	 * Returns a command for a three-score autonomous routine on SPEAKER left.
	 * Use two-score routine: Shoot a note, drive forward to the next note,
	 * intake, turn to align, and shoot the note. Play LEDs.
	 * Third note: Turn to middle note, drive forward, intake,
	 * turn to align (absolute -12 degrees), and shoot the note. End with LEDS.
	 * 
	 * @param driveSubsystem   The drive subsystem.
	 * @param arduinoSubsystem The arduino subsystem for LEDs.
	 * @param visionSubsystem  The vision subsystem with Limelight.
	 *                         If visionSubsystem is used, auto will include command
	 *                         to align with AprilTag.
	 * @return The command.
	 */
	public static Command getThreeScoreLeftAuto(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem,
			SimpleVisionSubsystem visionSubsystem) {
		SequentialCommandGroup alignCommand = new SequentialCommandGroup(
				new TurnToAngleCommand(driveSubsystem, 0, 2, false));
		if (visionSubsystem != null) {
			alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem));
		}
		return sequence(
				// right note
				getTwoScoreLeftAuto(driveSubsystem, arduinoSubsystem, visionSubsystem),
				// middle note
				new TurnToAngleCommand(driveSubsystem, -75, 2, false),
				new PolarDriveCommand(driveSubsystem, 1.25, -180, 0.01),
				alignCommand,
				new TimedLEDCommand(arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME));

	}

	/**
	 * Returns a command for a four-score autonomous routine on SPEAKER left.
	 * Use two-score routine: Shoot a note, drive forward to the next note,
	 * intake, turn to align, and shoot the note. Play LEDs.
	 * Third note: Turn to middle note, drive forward, intake,
	 * turn to align (absolute 12 degrees), and shoot the note. End with LEDS.
	 * 
	 * @param driveSubsystem   The drive subsystem.
	 * @param arduinoSubsystem The arduino subsystem for LEDs.
	 * @param visionSubsystem  The vision subsystem with Limelight.
	 *                         If visionSubsystem is used, auto will include command
	 *                         to align with AprilTag.
	 * @return The command.
	 */
	public static Command getFourScoreLeftAuto(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem,
			SimpleVisionSubsystem visionSubsystem) {
		SequentialCommandGroup alignCommand = new SequentialCommandGroup(
				new TurnToAngleCommand(driveSubsystem, -20, 2, false));
		if (visionSubsystem != null) {
			alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem));
		}
		return sequence(
				// left note and middle note
				getThreeScoreLeftAuto(driveSubsystem, arduinoSubsystem, visionSubsystem),
				// right note
				new PolarDriveCommand(driveSubsystem, 1.8, -300, 0.01),
				new TurnToAngleCommand(driveSubsystem, 0, false),
				new PolarDriveCommand(driveSubsystem, .5, -180, 0.01),
				alignCommand,
				new TimedLEDCommand(arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	/**
	 * Returns a command for a three-score autonomous routine on SPEAKER right.
	 * Use two-score routine: Shoot a note, drive forward to the next note,
	 * intake, turn to align, and shoot the note. Play LEDs.
	 * Third note: Turn to middle note, drive forward, intake,
	 * turn to align (absolute 12 degrees), and shoot the note. End with LEDS.
	 * 
	 * @param driveSubsystem   The drive subsystem.
	 * @param arduinoSubsystem The arduino subsystem for LEDs.
	 * @param visionSubsystem  The vision subsystem with Limelight.
	 *                         If visionSubsystem is used, auto will include command
	 *                         to align with AprilTag.
	 * @return The command.
	 */
	public static Command getFourScoreRightAuto(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem,
			SimpleVisionSubsystem visionSubsystem) {
		SequentialCommandGroup alignCommand = new SequentialCommandGroup(
				new TurnToAngleCommand(driveSubsystem, 20, 2, false));
		if (visionSubsystem != null) {
			alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem));
		}
		return sequence(
				// right note and middle note
				getThreeScoreRightAuto(driveSubsystem, arduinoSubsystem, visionSubsystem),
				// left note
				new PolarDriveCommand(driveSubsystem, 2, 300, 0.01),
				new TurnToAngleCommand(driveSubsystem, 0, false),
				new PolarDriveCommand(driveSubsystem, .5, 180, 0.01),
				alignCommand,
				new TimedLEDCommand(arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	/**
	 * Returns a command for
	 * 
	 * @param driveSubsystem   The drive subsystem.
	 * @param arduinoSubsystem The arduino subsystem for LEDs.
	 * @return The command.
	 */
	public static Command getAmpLeftAuto(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return sequence(
				// strafe towards speaker, shoot
				new PolarDriveCommand(driveSubsystem, 0.22, -90),
				new PolarDriveCommand(driveSubsystem, 0.4, 0),
				new TimedLEDCommand(arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME),
				// strafe away, intake
				new PolarDriveCommand(driveSubsystem, 1, -90),
				new PolarDriveCommand(driveSubsystem, 0.4, 180),
				// go back and shoot
				new PolarDriveCommand(driveSubsystem, 1, 90),
				new PolarDriveCommand(driveSubsystem, 0.4, 0),
				new TimedLEDCommand(arduinoSubsystem, 0.4, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	/**
	 * Returns a command for
	 * 
	 * @param driveSubsystem   The drive subsystem.
	 * @param arduinoSubsystem The arduino subsystem for LEDs.
	 * @return The command.
	 */
	public static Command getShootAndAmp(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return sequence(
				// strafe towards speaker, shoot
				new PolarDriveCommand(driveSubsystem, 1.4, -240, 0.01),
				new TurnToAngleCommand(driveSubsystem, 0, 2, false),
				new PolarDriveCommand(driveSubsystem, 0.4, 180, 0.01),
				// strafe away, intake
				new TurnToAngleCommand(driveSubsystem, 90, 2, false),
				new PolarDriveCommand(driveSubsystem, 1, -90),
				new PolarDriveCommand(driveSubsystem, 0.4, 180),
				// go back and shoot
				new PolarDriveCommand(driveSubsystem, 1, 90),
				new PolarDriveCommand(driveSubsystem, 0.4, 0),
				new TimedLEDCommand(arduinoSubsystem, 0.4, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	/**
	 * Returns a command to drive back and forth various amounts.
	 * 
	 * @param driveSubsystem   The drive subsystem.
	 * @param arduinoSubsystem The arduino subsystem for LEDs.
	 * @return The command.
	 */
	public static Command getBlocksAuto(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return sequence(
				DriveDistanceCommand.create(driveSubsystem, 0.75),
				DriveDistanceCommand.create(driveSubsystem, -1.25),
				DriveDistanceCommand.create(driveSubsystem, 1.5),
				DriveDistanceCommand.create(driveSubsystem, -1.825),
				DriveDistanceCommand.create(driveSubsystem, 2.125),
				DriveDistanceCommand.create(driveSubsystem, -2.5),
				new TimedLEDCommand(arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	public static Command getIntakeWithSensorCommand(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem,
			ArduinoSubsystem arduinoSubsystem) {
		return sequence(
				parallel(
						new IndexWithSensorCommand(indexerSubsystem, 0.5),
						intakeSubsystem.forwardIntakeCommand(),
						arduinoSubsystem.writeStatus(StatusCode.SOLID_ORANGE)),
				intakeSubsystem.stopIntakeCommand(),
				arduinoSubsystem.writeStatus(StatusCode.DEFAULT));
	}

	public static Command getTeleopIntakeCommand(IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, IndexerSubsystem indexerSubsystem,
			ArduinoSubsystem arduinoSubsystem) {
		return sequence(
				pneumaticsSubsystem.downIntakeCommand(),
				getIntakeWithSensorCommand(intakeSubsystem, indexerSubsystem, arduinoSubsystem),
				pneumaticsSubsystem.upIntakeCommand());
	}
}
