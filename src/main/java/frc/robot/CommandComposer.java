package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.PoseConstants.*;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.drive.DrivePathCommand;
import frc.robot.subsystems.AimerSubsystem;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.LimeLightSubsystem.Pose;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.SimpleVisionSubsystem;

public class CommandComposer {

	/**
	 * Runs a group of commands at the same time. Ends once all commands in the
	 * group finish.
	 *
	 * @param commands the commands to include
	 * @return the command
	 * @see ParallelCommandGroup
	 */
	public static Command parallel(Command... commands) {
		return new ParallelCommandGroup(commands);
	}

	public static Command getBlocksAuto(DriveSubsystem m_driveSubsystem) {
		return new SequentialCommandGroup(
				DriveDistanceCommand.create(m_driveSubsystem, 0.75),
				DriveDistanceCommand.create(m_driveSubsystem, -1.25),
				DriveDistanceCommand.create(m_driveSubsystem, 1.5),
				DriveDistanceCommand.create(m_driveSubsystem, -1.825),
				DriveDistanceCommand.create(m_driveSubsystem, 2.125),
				DriveDistanceCommand.create(m_driveSubsystem, -2.5));
	}

	public static Command getAlignToBlueAmpCommand(DriveSubsystem driveSubsystem,
			LimeLightSubsystem limeLightSubsystem) {
		return DriveCommand.alignTo(kBlueAmpPose.add(new Pose(0, -0.3, 0)), 0.2, 10, driveSubsystem, limeLightSubsystem)
				.andThen(DriveCommand.alignTo(kBlueAmpPose, 0.1, 5, driveSubsystem, limeLightSubsystem));
	}

	public static Command getAlignToRedAmpCommand(DriveSubsystem driveSubsystem,
			LimeLightSubsystem limeLightSubsystem) {
		return DriveCommand.alignTo(kRedAmpPose.add(new Pose(0, -0.3, 0)), 0.2, 10, driveSubsystem, limeLightSubsystem)
				.andThen(DriveCommand.alignTo(kRedAmpPose, 0.1, 5, driveSubsystem, limeLightSubsystem));
	}

	public static Command getTurnToBlueSpeakerCommand(DriveSubsystem driveSubsystem,
			LimeLightSubsystem limeLightSubsystem) {
		return DriveCommand.turnTo(kBlueSpeakerPosition, 0.1, 5, driveSubsystem, limeLightSubsystem);
	}

	public static Command getTurnToRedSpeakerCommand(DriveSubsystem driveSubsystem,
			LimeLightSubsystem limeLightSubsystem) {
		return DriveCommand.turnTo(kRedSpeakerPosition, 0.1, 5, driveSubsystem, limeLightSubsystem);
	}

	public static Command getTurnToClosestSpeakerCommand(DriveSubsystem driveSubsystem,
			LimeLightSubsystem limeLightSubsystem) {
		Supplier<Pose2d> s = () -> {
			var target = limeLightSubsystem.closest(kBlueSpeakerPosition, kRedSpeakerPosition);
			var t = limeLightSubsystem.transformationToward(target);
			return driveSubsystem.getPose().plus(t);
		};
		return new DriveCommand(driveSubsystem, s, 0.1, 5);
	}

	public static Command getMoveTowardClosestSpeakerCommand(double distanceToTarget, DriveSubsystem driveSubsystem,
			LimeLightSubsystem limeLightSubsystem) {
		Supplier<Pose2d> s = () -> {
			var target = limeLightSubsystem.closest(kBlueSpeakerPosition, kRedSpeakerPosition);
			var t = limeLightSubsystem.transformationToward(target, distanceToTarget);
			return driveSubsystem.getPose().plus(t);
		};
		return new DriveCommand(driveSubsystem, s, 0.1, 5);
	}

	public static Command getAlignToClosestAmpCommand(DriveSubsystem driveSubsystem,
			LimeLightSubsystem limeLightSubsystem) {
		Supplier<Pose2d> s1 = () -> {
			Pose target = new Pose(limeLightSubsystem.closest(kBlueAmpPose, kRedAmpPose));
			var t = limeLightSubsystem.transformationTo(target.add(new Pose(0, -0.3, 0)));
			return driveSubsystem.getPose().plus(t);
		};
		Supplier<Pose2d> s2 = () -> {
			var target = limeLightSubsystem.closest(kBlueAmpPose, kRedAmpPose);
			var t = limeLightSubsystem.transformationTo(target);
			return driveSubsystem.getPose().plus(t);
		};
		return new DriveCommand(driveSubsystem, s1, 0.1, 5).andThen(new DriveCommand(driveSubsystem, s2, 0.1, 5));
	}

	public static Command getMoveToBlueSpeakerCommand(DriveSubsystem driveSubsystem,
			LimeLightSubsystem limeLightSubsystem) {
		Supplier<Pose2d> s = () -> {
			var p = limeLightSubsystem.estimatedPose();
			var target = p;
			if (p.getY() > 2)
				target = new Pose(-6, 2, 180);
			else if (p.getY() < 0)
				target = new Pose(-6, 0, 180);
			return driveSubsystem.getPose().plus(target.minus(p));
		};
		return new DriveCommand(driveSubsystem, s, 0.1, 5)
				.andThen(DriveCommand.turnTo(kBlueSpeakerPosition, 0.1, 5, driveSubsystem,
						limeLightSubsystem));
	}

	public static Command getMoveToRedSpeakerCommand(DriveSubsystem driveSubsystem,
			LimeLightSubsystem limeLightSubsystem) {
		Supplier<Pose2d> s = () -> {
			var p = limeLightSubsystem.estimatedPose();
			var target = p;
			if (p.getY() > 2)
				target = new Pose(6, 2, 0);
			else if (p.getY() < 0)
				target = new Pose(6, 0, 0);
			return driveSubsystem.getPose().plus(target.minus(p));
		};
		return new DriveCommand(driveSubsystem, s, 0.1, 5)
				.andThen(DriveCommand.turnTo(kRedSpeakerPosition, 0.1, 5, driveSubsystem,
						limeLightSubsystem));
	}

	public static Command getShootToClosestSpeakerAtCommand(Pose2d targetPose, DriveSubsystem driveSubsystem,
			SimpleVisionSubsystem visionSubsystem, FlywheelSubsystem flywheelSubsystem, AimerSubsystem aimerSubsystem,
			IndexerSubsystem indexerSubsystem, Targeter targeter,
			LimeLightSubsystem limeLightSubsystem) {
		return DriveCommand.alignTo(targetPose, 0.1, 5, driveSubsystem,
				limeLightSubsystem).andThen(
						getAimAndShootAutoCommand(driveSubsystem,
								visionSubsystem, flywheelSubsystem, aimerSubsystem, indexerSubsystem,
								targeter, limeLightSubsystem));
	}

	public static Command getPickUpNoteAtCommand(Pose2d targetPose, DriveSubsystem driveSubsystem,
			SimpleVisionSubsystem visionSubsystem, FlywheelSubsystem flywheelSubsystem, AimerSubsystem aimerSubsystem,
			IndexerSubsystem indexerSubsystem, Targeter targeter,
			LimeLightSubsystem limeLightSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return DriveCommand
				.alignTo(targetPose.plus(new Transform2d(0.3, 0, Rotation2d.fromDegrees(0))), 0.1, 5, driveSubsystem,
						limeLightSubsystem)
				.andThen(parallel(
						DriveCommand.alignTo(targetPose, 0.1, 5, driveSubsystem,
								limeLightSubsystem),
						getTeleopIntakeCommand(intakeSubsystem, pneumaticsSubsystem, indexerSubsystem,
								arduinoSubsystem)));
	}

	public static Command getPickUpNoteAndShootAtCommand(Pose2d targetPose, DriveSubsystem driveSubsystem,
			SimpleVisionSubsystem visionSubsystem, FlywheelSubsystem flywheelSubsystem, AimerSubsystem aimerSubsystem,
			IndexerSubsystem indexerSubsystem, Targeter targeter,
			LimeLightSubsystem limeLightSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return getPickUpNoteAtCommand(targetPose, driveSubsystem,
				visionSubsystem, flywheelSubsystem, aimerSubsystem, indexerSubsystem,
				targeter, limeLightSubsystem, intakeSubsystem, pneumaticsSubsystem, arduinoSubsystem).andThen(
						getAimAndShootAutoCommand(driveSubsystem,
								visionSubsystem, flywheelSubsystem, aimerSubsystem, indexerSubsystem,
								targeter, limeLightSubsystem));
	}

	public static Command getFourScoreBlueAutoCommand(DriveSubsystem driveSubsystem,
			SimpleVisionSubsystem visionSubsystem, FlywheelSubsystem flywheelSubsystem, AimerSubsystem aimerSubsystem,
			IndexerSubsystem indexerSubsystem, Targeter targeter,
			LimeLightSubsystem limeLightSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return getShootToClosestSpeakerAtCommand(kBlueNoteThreePose.add(new Pose(-0.5, 0, 0)), driveSubsystem,
				visionSubsystem, flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem)
						.andThen( // 2nd note
								getPickUpNoteAndShootAtCommand(kBlueNoteThreePose, driveSubsystem,
										visionSubsystem, flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter,
										limeLightSubsystem, intakeSubsystem, pneumaticsSubsystem, arduinoSubsystem))
						.andThen( // 3rd note
								getPickUpNoteAndShootAtCommand(kBlueNoteTwoPose, driveSubsystem,
										visionSubsystem, flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter,
										limeLightSubsystem, intakeSubsystem, pneumaticsSubsystem, arduinoSubsystem))
						.andThen( // 4th note
								getPickUpNoteAndShootAtCommand(kBlueNoteOnePose, driveSubsystem,
										visionSubsystem, flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter,
										limeLightSubsystem, intakeSubsystem, pneumaticsSubsystem, arduinoSubsystem));
	}

	public static Command getFourScoreRedAutoCommand(DriveSubsystem driveSubsystem,
			SimpleVisionSubsystem visionSubsystem, FlywheelSubsystem flywheelSubsystem, AimerSubsystem aimerSubsystem,
			IndexerSubsystem indexerSubsystem, Targeter targeter,
			LimeLightSubsystem limeLightSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return getShootToClosestSpeakerAtCommand(kRedNoteThreePose.add(new Pose(0.5, 0, 0)), driveSubsystem,
				visionSubsystem, flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem)
						.andThen( // 2nd note
								getPickUpNoteAndShootAtCommand(kRedNoteThreePose, driveSubsystem,
										visionSubsystem, flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter,
										limeLightSubsystem, intakeSubsystem, pneumaticsSubsystem, arduinoSubsystem))
						.andThen( // 3rd note
								getPickUpNoteAndShootAtCommand(kRedNoteTwoPose, driveSubsystem,
										visionSubsystem, flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter,
										limeLightSubsystem, intakeSubsystem, pneumaticsSubsystem, arduinoSubsystem))
						.andThen( // 4th note
								getPickUpNoteAndShootAtCommand(kRedNoteOnePose, driveSubsystem,
										visionSubsystem, flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter,
										limeLightSubsystem, intakeSubsystem, pneumaticsSubsystem, arduinoSubsystem));
	}

	public static Command getFiveScoreBlueAutoCommand(DriveSubsystem driveSubsystem,
			SimpleVisionSubsystem visionSubsystem, FlywheelSubsystem flywheelSubsystem, AimerSubsystem aimerSubsystem,
			IndexerSubsystem indexerSubsystem, Targeter targeter,
			LimeLightSubsystem limeLightSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return getFourScoreBlueAutoCommand(driveSubsystem, visionSubsystem, flywheelSubsystem, aimerSubsystem,
				indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem, pneumaticsSubsystem, arduinoSubsystem)
						.andThen(getPickUpNoteAtCommand(kBlueCenterNoteOnePose, driveSubsystem, visionSubsystem,
								flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem,
								intakeSubsystem, pneumaticsSubsystem, arduinoSubsystem))
						.andThen(getShootToClosestSpeakerAtCommand(kBlueNoteOnePose, driveSubsystem,
								visionSubsystem, flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter,
								limeLightSubsystem));
	}

	public static Command getFiveScoreRedAutoCommand(DriveSubsystem driveSubsystem,
			SimpleVisionSubsystem visionSubsystem, FlywheelSubsystem flywheelSubsystem, AimerSubsystem aimerSubsystem,
			IndexerSubsystem indexerSubsystem, Targeter targeter,
			LimeLightSubsystem limeLightSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return getFourScoreRedAutoCommand(driveSubsystem, visionSubsystem, flywheelSubsystem, aimerSubsystem,
				indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem, pneumaticsSubsystem, arduinoSubsystem)
						.andThen(getPickUpNoteAtCommand(kRedCenterNoteOnePose, driveSubsystem, visionSubsystem,
								flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem,
								intakeSubsystem, pneumaticsSubsystem, arduinoSubsystem))
						.andThen(getShootToClosestSpeakerAtCommand(kRedNoteOnePose, driveSubsystem,
								visionSubsystem, flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter,
								limeLightSubsystem));
	}

	public static Command getFiveScoreBlueAutoCommandOptimized(DriveSubsystem driveSubsystem,
			LimeLightSubsystem limeLightSubsystem) {
		return
		// DriveCommand.turnTo(kBlueSpeakerPosition, 0.1, 5, driveSubsystem,
		// limeLightSubsystem)
		// 2nd note
		DriveCommand.alignTo(kBlueNoteThreePose, 0.1, 5, driveSubsystem,
				limeLightSubsystem)
				.andThen( // 3rd note
						DriveCommand.alignTo(kBlueNoteTwoPose, 0.1, 5, driveSubsystem,
								limeLightSubsystem))
				.andThen( // 4th note
						DriveCommand.alignTo(kBlueNoteOnePose, 0.1, 5, driveSubsystem,
								limeLightSubsystem))
				.andThen( // 5th note
						DrivePathCommand.passThrough(
								List.of(kBlueCenterNoteOnePose.add(new Pose(-0.5, 0, 0)), kBlueCenterNoteOnePose), 0.2,
								10,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueNoteOnePose, 0.1, 5, driveSubsystem,
								limeLightSubsystem))
		// .andThen(DriveCommand.turnTo(kBlueSpeakerPosition, 0.1, 5, driveSubsystem,
		// limeLightSubsystem))
		;
	}

	public static Command getFourScoreMiddleBlueAutoCommand(DriveSubsystem driveSubsystem,
			LimeLightSubsystem limeLightSubsystem) {
		return DriveCommand.alignTo(kBlueNoteOnePose, 0.1, 5, driveSubsystem,
				limeLightSubsystem)
				// .andThen(DriveCommand.turnTo(kBlueSpeakerPosition, 0.1, 5, driveSubsystem,
				// limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueCenterNoteOnePose.add(new Pose(-0.5, 0, 0)), 0.1, 5,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueCenterNoteOnePose, 0.1, 5, driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueNoteOnePose, 0.1, 5, driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueCenterNoteTwoPose.add(new Pose(-0.5, 0.5, 0)), 0.2, 10,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueCenterNoteTwoPose, 0.1, 5, driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueNoteOnePose, 0.1, 5, driveSubsystem,
								limeLightSubsystem))
		// .andThen(DriveCommand.turnTo(kBlueSpeakerPosition, 0.1, 5, driveSubsystem,
		// limeLightSubsystem))
		;
	}

	public static Command getFourScoreMiddleBlueAutoCommandOptimized(DriveSubsystem driveSubsystem,
			LimeLightSubsystem limeLightSubsystem) {
		return DriveCommand.alignTo(kBlueNoteOnePose, 0.1, 5, driveSubsystem,
				limeLightSubsystem)
				// .andThen(DriveCommand.turnTo(kBlueSpeakerPosition, 0.1, 5, driveSubsystem,
				// limeLightSubsystem))
				.andThen(
						DrivePathCommand.passThrough(
								List.of(kBlueCenterNoteOnePose.add(new Pose(-0.5, 0, 0)), kBlueCenterNoteOnePose), 0.1,
								5,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueNoteOnePose, 0.1, 5, driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DrivePathCommand.passThrough(
								List.of(kBlueCenterNoteTwoPose.add(new Pose(-0.5, 0.5, 0)), kBlueCenterNoteTwoPose),
								0.2, 10,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueNoteOnePose, 0.1, 5, driveSubsystem,
								limeLightSubsystem))
		// .andThen(DriveCommand.turnTo(kBlueSpeakerPosition, 0.1, 5, driveSubsystem,
		// limeLightSubsystem))
		;
	}

	public static Command getFourScoreMiddleCenterBlueAutoCommand(DriveSubsystem driveSubsystem,
			LimeLightSubsystem limeLightSubsystem) {
		return DriveCommand.alignTo(kBlueNoteTwoPose, 0.1, 5, driveSubsystem,
				limeLightSubsystem)
				// .andThen(DriveCommand.turnTo(kBlueSpeakerPosition, 0.1, 5, driveSubsystem,
				// limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueCenterNoteOnePose.add(new Pose(-0.5, 0, 0)), 0.1, 5,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueCenterNoteOnePose, 0.1, 5, driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueCenterNoteOnePose.add(new Pose(-2, 0, 0)), 0.1, 5,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueNoteTwoPose, 0.1, 5, driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueNoteTwoPose.add(new Pose(1.5, 0, 0)), 0.1, 5, driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueCenterNoteThreePose.add(new Pose(-0.5, 0, 0)), 0.2, 10,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueCenterNoteThreePose, 0.1, 5, driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueCenterNoteThreePose.add(new Pose(-2.75, 0, 0)), 0.2, 10,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueNoteTwoPose, 0.1, 5, driveSubsystem,
								limeLightSubsystem))
		// .andThen(DriveCommand.turnTo(kBlueSpeakerPosition, 0.1, 5, driveSubsystem,
		// limeLightSubsystem))
		;
	}

	public static Command getFourScoreMiddleCenterBlueAutoCommandOptimized(DriveSubsystem driveSubsystem,
			LimeLightSubsystem limeLightSubsystem) {
		return DriveCommand.alignTo(kBlueNoteTwoPose, 0.1, 5, driveSubsystem,
				limeLightSubsystem)
				// .andThen(DriveCommand.turnTo(kBlueSpeakerPosition, 0.1, 5, driveSubsystem,
				// limeLightSubsystem))
				.andThen(
						DrivePathCommand.passThrough(
								List.of(kBlueCenterNoteOnePose.add(new Pose(-0.5, 0, 0)), kBlueCenterNoteOnePose), 0.1,
								5,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DrivePathCommand.passThrough(
								List.of(kBlueCenterNoteOnePose.add(new Pose(-2, 0, 0)), kBlueNoteTwoPose), 0.1, 5,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DrivePathCommand.passThrough(
								List.of(kBlueNoteTwoPose.add(new Pose(1.5, 0, 0)),
										kBlueCenterNoteThreePose.add(new Pose(-0.5, 0, 0)), kBlueCenterNoteThreePose),
								0.2, 10,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DrivePathCommand.passThrough(
								List.of(kBlueCenterNoteThreePose.add(new Pose(-2.75, 0, 0)), kBlueNoteTwoPose), 0.2, 10,
								driveSubsystem,
								limeLightSubsystem))
		// .andThen(DriveCommand.turnTo(kBlueSpeakerPosition, 0.1, 5, driveSubsystem,
		// limeLightSubsystem))
		;
	}

	public static Command getFourScoreMiddleBottomBlueAutoCommand(DriveSubsystem driveSubsystem,
			LimeLightSubsystem limeLightSubsystem) {
		return DriveCommand.alignTo(kBlueNoteThreePose, 0.1, 5, driveSubsystem,
				limeLightSubsystem)
				// .andThen(DriveCommand.turnTo(kBlueSpeakerPosition, 0.1, 5, driveSubsystem,
				// limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueNoteThreePose.add(new Pose(0, -1, 0)), 0.1, 5,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueNoteThreePose.add(new Pose(2.3, -2.5, 0)), 0.1, 5,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueCenterNoteFourPose.add(new Pose(-.3, -.3, 0)), 0.1, 5,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueCenterNoteFourPose, 0.1, 5, driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueNoteThreePose.add(new Pose(2.3, -2.5, 0)), 0.1, 5,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueNoteThreePose.add(new Pose(1.5, -1.5, 0)), 0.1, 5,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueCenterNoteFivePose.add(new Pose(-.5, 0, 0)), 0.2, 10,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueCenterNoteFivePose, 0.1, 5, driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueNoteThreePose.add(new Pose(2.3, -2.5, 0)), 0.2, 10,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueNoteThreePose.add(new Pose(1.5, -1.5, 0)), 0.1, 5,
								driveSubsystem,
								limeLightSubsystem))
		// .andThen(DriveCommand.turnTo(kBlueSpeakerPosition, 0.1, 5, driveSubsystem,
		// limeLightSubsystem))
		;
	}

	public static Command getFourScoreMiddleBottomBlueAutoCommandOptimized(DriveSubsystem driveSubsystem,
			LimeLightSubsystem limeLightSubsystem) {
		return DriveCommand.alignTo(kBlueNoteThreePose, 0.1, 5, driveSubsystem,
				limeLightSubsystem)
				// .andThen(DriveCommand.turnTo(kBlueSpeakerPosition, 0.1, 5, driveSubsystem,
				// limeLightSubsystem))
				.andThen(
						DrivePathCommand.passThrough(
								List.of(kBlueNoteThreePose.add(new Pose(0, -1, 0)),
										kBlueNoteThreePose.add(new Pose(2.3, -2.5, 0)),
										kBlueCenterNoteFourPose.add(new Pose(-.3, -.3, 0)), kBlueCenterNoteFourPose),
								0.1, 5,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DrivePathCommand.passThrough(
								List.of(kBlueNoteThreePose.add(new Pose(2.3, -2.5, 0)),
										kBlueNoteThreePose.add(new Pose(1.5, -1.5, 0))),
								0.1, 5,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DrivePathCommand.passThrough(
								List.of(kBlueCenterNoteFivePose.add(new Pose(-.5, 0, 0)), kBlueCenterNoteFivePose),
								0.2, 10,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DrivePathCommand.passThrough(List.of(kBlueNoteThreePose.add(new Pose(2.3, -2.5, 0)),
								kBlueNoteThreePose.add(new Pose(1.5, -1.5, 0))), 0.2, 10, driveSubsystem,
								limeLightSubsystem))
		// .andThen(DriveCommand.turnTo(kBlueSpeakerPosition, 0.1, 5, driveSubsystem,
		// limeLightSubsystem))
		;
	}

	private static Command getAimAndShootAutoCommand(DriveSubsystem driveSubsystem,
			SimpleVisionSubsystem visionSubsystem, FlywheelSubsystem flywheelSubsystem, AimerSubsystem aimerSubsystem,
			IndexerSubsystem indexerSubsystem, Targeter targeter, LimeLightSubsystem limeLightSubsystem) {
		return getTurnToClosestSpeakerCommand(driveSubsystem, limeLightSubsystem).andThen(new WaitCommand(0.0) {
			@Override
			public void end(boolean interrupted) {
				System.out.println("shot to speaker");
			}
		});
	}

	private static Command getTeleopIntakeCommand(IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, IndexerSubsystem indexerSubsystem,
			ArduinoSubsystem arduinoSubsystem) {
		return new WaitCommand(0.0) {
			@Override
			public void end(boolean interrupted) {
				System.out.println("note picked up");
			}
		};
	}

	public static Command getTwoMiddleFourScoreRedCommand(DriveSubsystem driveSubsystem,
			SimpleVisionSubsystem visionSubsystem, FlywheelSubsystem flywheelSubsystem, AimerSubsystem aimerSubsystem,
			IndexerSubsystem indexerSubsystem, Targeter targeter,
			LimeLightSubsystem limeLightSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return sequence(

				getAimAndShootAutoCommand(driveSubsystem, visionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, targeter, limeLightSubsystem),
				getPickUpNoteAndShootAtCommand(kRedNoteThreePose, driveSubsystem, visionSubsystem, flywheelSubsystem,
						aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem,
						pneumaticsSubsystem, arduinoSubsystem),
				DriveCommand.moveToward(new Translation2d(6, -2), 0, 0.5, 360, driveSubsystem, limeLightSubsystem),
				getPickUpNoteAtCommand(kRedCenterNoteFivePose, driveSubsystem, visionSubsystem, flywheelSubsystem,
						aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem,
						pneumaticsSubsystem, arduinoSubsystem),
				DriveCommand.moveToward(new Translation2d(6, -2), 0, 0.5, 360, driveSubsystem, limeLightSubsystem),
				getShootToClosestSpeakerAtCommand(new Pose(6, -0.5, 25), driveSubsystem, visionSubsystem,
						flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem),
				DriveCommand.moveToward(new Translation2d(6, -3), 0, 0.5, 360, driveSubsystem, limeLightSubsystem),
				DriveCommand.moveToward(new Translation2d(2, -3), 0, 0.5, 360, driveSubsystem, limeLightSubsystem),
				getPickUpNoteAtCommand(kRedCenterNoteFourPose, driveSubsystem, visionSubsystem, flywheelSubsystem,
						aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem,
						pneumaticsSubsystem, arduinoSubsystem),
				DriveCommand.moveToward(new Translation2d(6, -3), 0, 0.5, 360, driveSubsystem, limeLightSubsystem),
				getShootToClosestSpeakerAtCommand(new Pose(6, -0.5, 25), driveSubsystem, visionSubsystem,
						flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem)

		);
	}

	public static Command getTwoMiddleFourScoreBlueCommand(DriveSubsystem driveSubsystem,
			SimpleVisionSubsystem visionSubsystem, FlywheelSubsystem flywheelSubsystem, AimerSubsystem aimerSubsystem,
			IndexerSubsystem indexerSubsystem, Targeter targeter,
			LimeLightSubsystem limeLightSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return sequence(

				getAimAndShootAutoCommand(driveSubsystem, visionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, targeter, limeLightSubsystem),
				getPickUpNoteAndShootAtCommand(kBlueNoteThreePose, driveSubsystem, visionSubsystem, flywheelSubsystem,
						aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem,
						pneumaticsSubsystem, arduinoSubsystem),
				DriveCommand.moveToward(new Translation2d(-6, -2), 0, 0.5, 360, driveSubsystem, limeLightSubsystem),
				getPickUpNoteAtCommand(kBlueCenterNoteFivePose, driveSubsystem, visionSubsystem, flywheelSubsystem,
						aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem,
						pneumaticsSubsystem, arduinoSubsystem),
				DriveCommand.moveToward(new Translation2d(-6, -2), 0, 0.5, 360, driveSubsystem, limeLightSubsystem),
				getShootToClosestSpeakerAtCommand(new Pose(-6, -0.5, 155), driveSubsystem, visionSubsystem,
						flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem),
				DriveCommand.moveToward(new Translation2d(-6, -3), 0, 0.5, 360, driveSubsystem, limeLightSubsystem),
				DriveCommand.moveToward(new Translation2d(-2, -3), 0, 0.5, 360, driveSubsystem, limeLightSubsystem),
				getPickUpNoteAtCommand(kBlueCenterNoteFourPose, driveSubsystem, visionSubsystem, flywheelSubsystem,
						aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem,
						pneumaticsSubsystem, arduinoSubsystem),
				DriveCommand.moveToward(new Translation2d(-6, -3), 0, 0.5, 360, driveSubsystem, limeLightSubsystem),
				getShootToClosestSpeakerAtCommand(new Pose(-6, -0.5, 155), driveSubsystem, visionSubsystem,
						flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem)

		);
	}

	// TODO: NOT TESTED YET
	public static Command getFiveScoreTwoAndTwoBlueCommand(DriveSubsystem driveSubsystem,
			SimpleVisionSubsystem visionSubsystem, FlywheelSubsystem flywheelSubsystem, AimerSubsystem aimerSubsystem,
			IndexerSubsystem indexerSubsystem, Targeter targeter,
			LimeLightSubsystem limeLightSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return sequence(
				getAimAndShootAutoCommand(driveSubsystem, visionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, targeter, limeLightSubsystem),
				getPickUpNoteAndShootAtCommand(kBlueNoteTwoPose, driveSubsystem, visionSubsystem, flywheelSubsystem,
						aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem,
						pneumaticsSubsystem, arduinoSubsystem),
				getPickUpNoteAndShootAtCommand(kBlueNoteOnePose, driveSubsystem, visionSubsystem, flywheelSubsystem,
						aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem,
						pneumaticsSubsystem, arduinoSubsystem),
				DriveCommand.alignTo(new Pose(-5, 3, 180 + 25), 1, 10, driveSubsystem, limeLightSubsystem),
				getPickUpNoteAtCommand(kBlueCenterNoteOnePose, driveSubsystem, visionSubsystem, flywheelSubsystem,
						aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem,
						pneumaticsSubsystem, arduinoSubsystem),
				DriveCommand.alignTo(new Pose(-6.5, 2.5, 180 + 25), 1, 10, driveSubsystem, limeLightSubsystem),
				getAimAndShootAutoCommand(driveSubsystem, visionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, targeter, limeLightSubsystem),
				DriveCommand.alignTo(new Pose(-3, 3, 180 + 25), 1, 10, driveSubsystem, limeLightSubsystem),
				getPickUpNoteAtCommand(kBlueCenterNoteTwoPose, driveSubsystem, visionSubsystem, flywheelSubsystem,
						aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem,
						pneumaticsSubsystem, arduinoSubsystem),
				DriveCommand.alignTo(new Pose(-6.5, 2.5, 180 + 25), 0.5, 10, driveSubsystem, limeLightSubsystem),
				getAimAndShootAutoCommand(driveSubsystem, visionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, targeter, limeLightSubsystem));
	}

	// TODO: NOT TESTED YET
	public static Command getFiveScoreTwoAndTwoRedCommand(DriveSubsystem driveSubsystem,
			SimpleVisionSubsystem visionSubsystem, FlywheelSubsystem flywheelSubsystem, AimerSubsystem aimerSubsystem,
			IndexerSubsystem indexerSubsystem, Targeter targeter,
			LimeLightSubsystem limeLightSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return sequence(
				getAimAndShootAutoCommand(driveSubsystem, visionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, targeter, limeLightSubsystem),
				getPickUpNoteAndShootAtCommand(kRedNoteTwoPose, driveSubsystem, visionSubsystem, flywheelSubsystem,
						aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem,
						pneumaticsSubsystem, arduinoSubsystem),
				getPickUpNoteAndShootAtCommand(kRedNoteOnePose, driveSubsystem, visionSubsystem, flywheelSubsystem,
						aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem,
						pneumaticsSubsystem, arduinoSubsystem),
				DriveCommand.alignTo(new Pose(5, 3, -25), 1, 10, driveSubsystem, limeLightSubsystem),
				getPickUpNoteAtCommand(kRedCenterNoteOnePose, driveSubsystem, visionSubsystem, flywheelSubsystem,
						aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem,
						pneumaticsSubsystem, arduinoSubsystem),
				DriveCommand.alignTo(new Pose(6.5, 2.5, -25), 1, 10, driveSubsystem, limeLightSubsystem),
				getAimAndShootAutoCommand(driveSubsystem, visionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, targeter, limeLightSubsystem),
				DriveCommand.alignTo(new Pose(3, 3, -25), 1, 10, driveSubsystem, limeLightSubsystem),
				getPickUpNoteAtCommand(kRedCenterNoteTwoPose, driveSubsystem, visionSubsystem, flywheelSubsystem,
						aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem,
						pneumaticsSubsystem, arduinoSubsystem),
				DriveCommand.alignTo(new Pose(6.5, 2.5, -25), 0.5, 10, driveSubsystem, limeLightSubsystem),
				getAimAndShootAutoCommand(driveSubsystem, visionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, targeter, limeLightSubsystem));
	}

	public static Command getThreeScoreTwoMiddleBottomBlueAuto(DriveSubsystem driveSubsystem,
			SimpleVisionSubsystem visionSubsystem, FlywheelSubsystem flywheelSubsystem, AimerSubsystem aimerSubsystem,
			IndexerSubsystem indexerSubsystem, Targeter targeter,
			LimeLightSubsystem limeLightSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return sequence(
				getAimAndShootAutoCommand(driveSubsystem, visionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, targeter, limeLightSubsystem),
				DriveCommand.alignTo(new Pose(-5, -4, 180 + 25), 0.3, 10, driveSubsystem, limeLightSubsystem),
				getPickUpNoteAtCommand(kBlueCenterNoteFourPose, driveSubsystem, visionSubsystem, flywheelSubsystem,
						aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem,
						pneumaticsSubsystem, arduinoSubsystem),
				DriveCommand.alignTo(new Pose(-5.5, -3, 180 + 25), 0.3, 10, driveSubsystem, limeLightSubsystem),
				DriveCommand.alignTo(new Pose(-6.25, 1, 180 + 25), 0.3, 10, driveSubsystem, limeLightSubsystem),
				getAimAndShootAutoCommand(driveSubsystem, visionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, targeter, limeLightSubsystem),
				DriveCommand.alignTo(new Pose(-5.5, -3, 180 + 25), 0.3, 10, driveSubsystem, limeLightSubsystem),
				getPickUpNoteAtCommand(kBlueCenterNoteFivePose, driveSubsystem, visionSubsystem, flywheelSubsystem,
						aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem,
						pneumaticsSubsystem, arduinoSubsystem),
				DriveCommand.alignTo(new Pose(-5.5, -3, 180 + 25), 0.3, 10, driveSubsystem, limeLightSubsystem),
				DriveCommand.alignTo(new Pose(-6.25, 1, 180 + 25), 0.3, 10, driveSubsystem, limeLightSubsystem),

				getAimAndShootAutoCommand(driveSubsystem, visionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, targeter, limeLightSubsystem));
	}

	public static Command getThreeScoreTwoMiddleBottomRedAuto(DriveSubsystem driveSubsystem,
			SimpleVisionSubsystem visionSubsystem, FlywheelSubsystem flywheelSubsystem, AimerSubsystem aimerSubsystem,
			IndexerSubsystem indexerSubsystem, Targeter targeter,
			LimeLightSubsystem limeLightSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return sequence(
				getAimAndShootAutoCommand(driveSubsystem, visionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, targeter, limeLightSubsystem),
				DriveCommand.alignTo(new Pose(5, -4, -25), 0.3, 10, driveSubsystem, limeLightSubsystem),
				getPickUpNoteAtCommand(kRedCenterNoteFourPose, driveSubsystem, visionSubsystem, flywheelSubsystem,
						aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem,
						pneumaticsSubsystem, arduinoSubsystem),
				DriveCommand.alignTo(new Pose(5.5, -3, -25), 0.3, 10, driveSubsystem, limeLightSubsystem),
				DriveCommand.alignTo(new Pose(6.25, 1, -25), 0.3, 10, driveSubsystem, limeLightSubsystem),
				getAimAndShootAutoCommand(driveSubsystem, visionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, targeter, limeLightSubsystem),
				DriveCommand.alignTo(new Pose(5.5, -3, -25), 0.3, 10, driveSubsystem, limeLightSubsystem),
				getPickUpNoteAtCommand(kRedCenterNoteFivePose, driveSubsystem, visionSubsystem, flywheelSubsystem,
						aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem,
						pneumaticsSubsystem, arduinoSubsystem),
				DriveCommand.alignTo(new Pose(5.5, -3, -25), 0.3, 10, driveSubsystem, limeLightSubsystem),
				DriveCommand.alignTo(new Pose(6.25, 1, -25), 0.3, 10, driveSubsystem, limeLightSubsystem),

				getAimAndShootAutoCommand(driveSubsystem, visionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, targeter, limeLightSubsystem));
	}

}
