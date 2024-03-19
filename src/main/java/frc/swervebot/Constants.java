// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.swervebot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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
		public static final double kDeadzone = 0.1;
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
		public static final int kCounterWeightPort = 17;
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
		public static final double kDriveScale = 0.5;
		// Drive PID values
		public static final double kP = 0.005;
		public static final double kI = 0.045;
		public static final double kD = 0;
		public static final double kIz = 5;
		public static final double kFF = 0;
		public static final double kMinOutput = -1.0;
		public static final double kMaxOutput = 1.0;

		public static final double kDriveP = 0.4; // up to 1.0?
		public static final double kDriveI = 0;
		public static final double kDriveD = 0;
		public static final double kDriveMaxVelocity = 2;
		public static final double kDriveMaxAcceleration = 4; // up to 10?

		public static final double kTurnP = 0.02; // was 0.005 upto 0.2?
		public static final double kTurnI = 0; // was 0.003
		public static final double kTurnD = 0; // 0.0
		public static final double kTurnMaxVelocity = 120;
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
		public static final double kModuleResponseTimeSeconds = 0.02;
		public static final double kGearRatio = 6.12;
		public static final double kWheelDiameter = Units.inchesToMeters(4);

		public static final double kMotorRotationsPerMeter = (1 / kGearRatio) * (Math.PI * kWheelDiameter);

		public static final Translation2d kFrontLeftLocation = new Translation2d(-0.381, 0.381); // -+
		public static final Translation2d kFrontRightLocation = new Translation2d(0.381, 0.381); // ++
		public static final Translation2d kBackLeftLocation = new Translation2d(-0.381, -0.381); // --
		public static final Translation2d kBackRightLocation = new Translation2d(0.381, -0.381); // +-

		public static final int kDriveSmartCurrentLimit = 55;
		public static final int kDrivePeakCurrentLimit = 65;
		public static final int kSteerSmartCurrentLimit = 30;
		public static final int kSteerPeakCurrentLimit = 35;

		// The amount of time to go from 0 to full power in seconds
		public static final double kRampRate = .1;
	}

	public static final class PoseConstants {
		public static final Translation2d kBlueSpeakerPosition = new Translation2d(-7.87, 1.45);
		public static final Pose kBlueAmpPose = new Pose(-6.44, 3.75, 90);

		public static final Pose kBlueNoteOnePose = new Pose(-6.0 + 0.55, 2.82 - 0.25 + 0.4, 180 + 25);
		public static final Pose kBlueNoteTwoPose = new Pose(-6.0 + 0.55 + 0.2, 1.65, 180);
		// to avoid collision
		public static final Pose kBlueNoteThreePose = new Pose(-6.0 + 0.45 - 0.2, 0.0 + 0.45 - 0.3, 180 - 25);

		public static final Pose kBlueCenterNoteOnePose = new Pose(-0.2 + 0.2, 3.4, 180);
		public static final Pose kBlueCenterNoteTwoPose = new Pose(-0.2 + 0.25 + 0.4, 1.7 + 0.25, 180 - 45);
		public static final Pose kBlueCenterNoteThreePose = new Pose(-0.2 + 0.4, 0, 180);
		public static final Pose kBlueCenterNoteFourPose = new Pose(-0.2 + 0.25 + 0.4, -1.7 - 0.05, 180 + 30);
		public static final Pose kBlueCenterNoteFivePose = new Pose(-0.2 + 0.4, -3.4 + 0.5, 180 - 15);

		public static final Translation2d kRedSpeakerPosition = new Translation2d(7.87, 1.45);
		public static final Pose kRedAmpPose = new Pose(6.44, 3.75, 90);

		public static final Pose kRedNoteOnePose = new Pose(6.0 - 0.55, 2.82 - 0.25 + 0.4, -25);
		public static final Pose kRedNoteTwoPose = new Pose(6.0 - 0.55 + 0.2, 1.65, 0);
		// to avoid collision
		public static final Pose kRedNoteThreePose = new Pose(6.0 - 0.45 + 0.2, 0.0 + 0.45 - 0.3, 25);

		public static final Pose kRedCenterNoteOnePose = new Pose(0.2 - 0.2, 3.4, 0);
		public static final Pose kRedCenterNoteTwoPose = new Pose(0.2 - 0.25 - 0.4, 1.7 + 0.25, 45);
		public static final Pose kRedCenterNoteThreePose = new Pose(0.2 - 0.4, 0, 0);
		public static final Pose kRedCenterNoteFourPose = new Pose(0.2 - 0.25 - 0.4, -1.7 - 0.05, -30);
		public static final Pose kRedCenterNoteFivePose = new Pose(0.2 - 0.4, -3.4 + 0.5, 0 + 15);
	}
}