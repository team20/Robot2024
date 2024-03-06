// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.subsystems.LimeLightSubsystem.Pose;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static final class ControllerConstants {
		public static final int kDriverControllerPort = 0;
		public static final int kOperatorControllerPort = 1;
		public static final double kDeadzone = 0.05;
		public static final double kTriggerDeadzone = .05;

		public static final class Axis {
			public static final int kLeftX = 0;
			public static final int kLeftY = 1;
			public static final int kRightX = 2;
			public static final int kLeftTrigger = 3;
			public static final int kRightTrigger = 4;
			public static final int kRightY = 5;
		}

		public static final class Button {
			/** Left middle button */
			public static final int kSquare = 1;
			/** Bottom button */
			public static final int kX = 2;
			/** Right middle button */
			public static final int kCircle = 3;
			/** Top button */
			public static final int kTriangle = 4;
			public static final int kLeftBumper = 5;
			public static final int kRightBumper = 6;
			public static final int kLeftTrigger = 7;
			public static final int kRightTrigger = 8;
			public static final int kShare = 9;
			public static final int kOptions = 10;
			public static final int kLeftStick = 11;
			public static final int kRightStick = 12;
			public static final int kPS = 13;
			public static final int kTrackpad = 14;
		}

		public static final class DPad {
			public static final int kUp = 0;
			public static final int kRight = 90;
			public static final int kDown = 180;
			public static final int kLeft = 270;
		}
	}

	public static final class DriveConstants {
		// CAN IDs (updated)
		public static final int kFrontRightDrivePort = 10;
		public static final int kFrontRightSteerPort = 11;
		public static final int kFrontLeftDrivePort = 40;
		public static final int kFrontLeftSteerPort = 41;
		public static final int kBackRightDrivePort = 20;
		public static final int kBackRightSteerPort = 21;
		public static final int kBackLeftDrivePort = 30;
		public static final int kBackLeftSteerPort = 31;
		public static final int kFrontRightCANCoderPort = 12;
		public static final int kFrontLeftCANCoderPort = 42;
		public static final int kBackRightCANCoderPort = 22;
		public static final int kBackLeftCANCoderPort = 32;
		// Swerve PID values
		public static final double kP = 0.005;
		public static final double kI = 0.045;
		public static final double kD = 0;
		public static final double kIz = 5;

		public static final double kDriveP = 0.4; // up to 1.0?
		public static final double kDriveI = 0;
		public static final double kDriveD = 0;
		public static final double kDriveMaxVelocity = 3; // up to 5?
		public static final double kDriveMaxAcceleration = 3; // up to 10?

		public static final double kTurnP = 0.02; // was 0.005 upto 0.2?
		public static final double kTurnI = 0; // was 0.003
		public static final double kTurnD = 0; // 0.0
		public static final double kTurnMaxVelocity = 120; // up to 240?
		public static final double kTurnMaxAcceleration = 240; // up to 360?

		/*** Distance between center of front wheel and center of back wheel */
		public static final double kWheelBase = 21.5;
		/*** Distance between center of left wheel and center of right wheel */
		public static final double kTrackWidth = 21.5;
		public static final double kSteerPeriod = 0.02;
		// Speed multiplier to make sure the robot doesn't crash into something when
		// testing, because crashing into people's shins would be bad
		public static final double kMaxSpeed = 1;
		public static final double kMinSpeed = 0.1;
		public static final double kTeleopMaxSpeed = 1;
		public static final double kTeleopMaxTurnSpeed = 0.6;
		public static final double kModuleResponseTimeSeconds = 0.02;
		public static final double kGearRatio = 6.12;
		public static final double kWheelDiameter = Units.inchesToMeters(4);

		public static final double kMotorRotationsPerMeter = (1 / kGearRatio) * (Math.PI * kWheelDiameter);

		public static final Translation2d kFrontLeftLocation = new Translation2d(0.381, 0.381); // --
		public static final Translation2d kFrontRightLocation = new Translation2d(0.381, -0.381); // +-
		public static final Translation2d kBackLeftLocation = new Translation2d(-0.381, 0.381); // ++
		public static final Translation2d kBackRightLocation = new Translation2d(-0.381, -0.381); // -+

		public static final int kDriveSmartCurrentLimit = 55;
		public static final int kDrivePeakCurrentLimit = 65;
		public static final int kSteerSmartCurrentLimit = 30;
		public static final int kSteerPeakCurrentLimit = 35;

		// The amount of time to go from 0 to full power in seconds
		public static final double kRampRate = .1; // .1

	}

	public static final class FlywheelConstants {
		public static final int kBottomPort = 50;
		public static final int kTopPort = 49;
		public static final boolean kBottomInvert = true;
		public static final boolean kTopInvert = true;
		public static final int kSmartCurrentLimit = 50;
		public static final double kPeakCurrentLimit = 60;
		public static final int kPeakCurrentDurationMillis = 100;
		public static final double kP = 0.000_1;
		public static final double kI = 0;
		public static final double kD = 0;
		public static final double kIz = 0.0;
		public static final double kFF = .000_1050;
		public static final double kMaxOutput = 1;
		public static final double kMinOutput = 0;
		public static final double kGearRatio = 2;
		public static final double kAllowedError = 50;
	}

	public static final class ClimbConstants {
		public static final double kFF = 0.00008040000102482736;
		public static final int kLeftPort = 45;
		public static final int kRightPort = 44;
		public static final boolean kLeftInvert = false;
		public static final boolean kRightInvert = false;
		public static final int kSmartCurrentLimit = 60;
		public static final int kSecondaryCurrentLimit = 70;
		public static final double kMinOutput = -1;
		public static final double kMaxOutput = 1;
		public static final double kP = 0.3;
		public static final double kI = 0;
		public static final double kD = 0;
		public static final double kTolerance = 1;
		public static final int kMaxExtension = 250;
	}

	public static final class IntakeConstants {
		public static final int kIntakePort = 46;
		public static final boolean kInvert = true;
		public static final int kSmartCurrentLimit = 60;
		public static final int kPeakCurrentLimit = 60;
		public static final boolean kFollowerOppose = false;
		public static final double kIntakeGearRatio = 1.0 / 10.0;
		public static final double kIntakeSpeed = 0.75;
		public static final double kReverseSpeed = -0.4;
	}

	public static final class IndexerConstants {
		public static final int kIndexerPort = 47;
		public static final boolean kInvert = true;
		public static final int kIndexerPeakCurrentLimit = 60;
		public static final int kIndexerSmartCurrentLimit = 55;
		public static final double kIndexerMaxSpeed = 1;
		public static final double kIndexerMinSpeed = 0.1;

		// Shoot Command Constants
		public static final double kKickTime = 2;
		public static final double kKickSpeed = 0.3;
		public static final double kIndexerGearRatio = 1.0 / 4.0;
		public static final double kIntakeSpeed = 0.2;
		public static final double kReverseSpeed = -0.3;
	}

	public static final class PneumaticsConstants {
		public static final int kPneumaticHubID = 5;
		public static final int kLeftAmpBarForwardChannel = 4;
		public static final int kLeftAmpBarReverseChannel = 5;
		public static final int kRightAmpBarForwardChannel = 6;
		public static final int kRightAmpBarReverseChannel = 7;
		public static final int kLeftIntakeForwardChannel = 0;
		public static final int kLeftIntakeReverseChannel = 1;
		public static final int kRightIntakeForwardChannel = 2;
		public static final int kRightIntakeReverseChannel = 3;
		// TODO direction/starting state?
		/** Alias for the solenoid value that makes the intake go down. */
		public static final Value kIntakeDown = Value.kForward;
		/** Alias for the solenoid value that makes the intake go up. */
		public static final Value kIntakeUp = Value.kReverse;
	}

	public static final class AimerConstants {
		public static final double kAimerMaxEncoderValue = 0.268;
		public static final int kAimerLeadScrewPort = 48;
		public static final int kAimerEncoderPort = 51;
		public static final boolean kMasterInvert = false;
		public static final int kSmartCurrentLimit = 25; // 90 (NO MORE THAN 25 A)
		public static final double kPeakCurrentLimit = 35; // 100 (NO MORE THAN 40 A)
		public static final int kPeakCurrentDurationMillis = 100;
		public static final double kGearRatio = 3;
		public static final double kSpeakerHeight = 2.032145; // TODO change to this height - robot height
		public static final double kAimerLength = .25; // TODO change to deltaX between edge of robot and aimer axis
														// on robot
		public static final double kDefaultActuatorHeight = 0.973; // percent
		public static final double kSubwooferActuatorHeight = 0.973;
		public static final double kAmpActuatorHeight = 0.964720;
		public static final double kAimerTolerance = 0.001; // previously 0.01
		public static final double kAdjustAmount = 0.05; // Adjust with testing
		public static final double kP = 5;
		public static final double kMaxAimerPower = 1;
		public static final double kMinAimerPower = 0.1;
	}

	public static final class PoseConstants {
		public static final Translation2d kBlueSpeakerPosition = new Translation2d(-7.87, 1.45);
		public static final Pose kBlueAmpPose = new Pose(-6.44, 3.75, 90);
		public static final Pose kBlueNoteOnePose = new Pose(-6.0 + 0.25, 2.82 - 0.25, 180 + 25);
		public static final Pose kBlueNoteTwoPose = new Pose(-6.0 + 0.15, 1.45, 180);
		public static final Pose kBlueNoteThreePose = new Pose(-6.0 + 0.25, 0.0 + 0.25, 180 - 25);
		public static final Pose kBlueCenterNoteOnePose = new Pose(-0.2, 3.2, 180);
		public static final Pose kBlueCenterNoteTwoPose = new Pose(-0.2 + 0.25, 1.7 + 0.25, 180 - 45);
		public static final Pose kBlueCenterNoteThreePose = new Pose(-0.2, 0, 180);
		public static final Pose kBlueCenterNoteFourPose = new Pose(-0.2 + 0.25, -1.7 - 0.15, 180 + 45);
		public static final Pose kBlueCenterNoteFivePose = new Pose(-0.2, -3.2, 180);

		public static final Translation2d kRedSpeakerPosition = new Translation2d(7.87, 1.45);
		public static final Pose kRedAmpPose = new Pose(6.44, 3.75, 90);
		public static final Pose kRedNoteOnePose = new Pose(6.0 - 0.25, 2.82 - 0.25, -25);
		public static final Pose kRedNoteTwoPose = new Pose(6.0 - 0.15, 1.45, 0);
		public static final Pose kRedNoteThreePose = new Pose(6.0 - 0.25, 0.0 + 0.25, 25);
		public static final Pose kRedCenterNoteOnePose = new Pose(0.2, 3.2, 0);
		public static final Pose kRedCenterNoteTwoPose = new Pose(0.2 - 0.25, 1.7 + 0.25, 45);
		public static final Pose kRedCenterNoteThreePose = new Pose(0.2, 0, 0);
		public static final Pose kRedCenterNoteFourPose = new Pose(0.2 - 0.25, -1.7 - 0.15, -45);
		public static final Pose kRedCenterNoteFivePose = new Pose(0.2, -3.2, 0);
	}
}