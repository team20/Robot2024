// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.ProtobufPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ControllerConstants;
import frc.robot.SwerveModule;

public class DriveSubsystem extends SubsystemBase {
	private final SwerveModule m_frontLeft;
	private final SwerveModule m_frontRight;
	private final SwerveModule m_backLeft;
	private final SwerveModule m_backRight;

	private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
			kFrontLeftLocation, kFrontRightLocation, kBackLeftLocation, kBackRightLocation);
	private final SwerveDriveOdometry m_odometry;
	private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
	private Rotation2d m_heading = new Rotation2d();
	private final SysIdRoutine m_sysidRoutine;

	private final ProtobufPublisher<Pose2d> m_posePublisher;
	private final StructArrayPublisher<SwerveModuleState> m_targetModuleStatePublisher;
	private final StructArrayPublisher<SwerveModuleState> m_currentModuleStatePublisher;

	/** Creates a new DriveSubsystem. */
	public DriveSubsystem() {
		m_posePublisher = NetworkTableInstance.getDefault().getProtobufTopic("/SmartDashboard/Pose", Pose2d.proto)
				.publish();
		m_targetModuleStatePublisher = NetworkTableInstance.getDefault()
				.getStructArrayTopic("/SmartDashboard/Target Swerve Modules States", SwerveModuleState.struct)
				.publish();
		m_currentModuleStatePublisher = NetworkTableInstance.getDefault()
				.getStructArrayTopic("/SmartDashboard/Current Swerve Modules States", SwerveModuleState.struct)
				.publish();
		m_frontLeft = new SwerveModule(kFrontLeftCANCoderPort, kFrontLeftDrivePort, kFrontLeftSteerPort);
		m_frontRight = new SwerveModule(kFrontRightCANCoderPort, kFrontRightDrivePort, kFrontRightSteerPort);
		m_backLeft = new SwerveModule(kBackLeftCANCoderPort, kBackLeftDrivePort, kBackLeftSteerPort);
		m_backRight = new SwerveModule(kBackRightCANCoderPort, kBackRightDrivePort, kBackRightSteerPort);
		var config = new SysIdRoutine.Config(Units.Volts.of(2.5).per(Units.Seconds.of(1)), null, Units.Seconds.of(3));
		m_sysidRoutine = new SysIdRoutine(config, new SysIdRoutine.Mechanism(volt -> {
			var state = new SwerveModuleState(volt.magnitude(), new Rotation2d(Math.PI / 2));
			m_frontLeft.setModuleState(state);
			m_frontRight.setModuleState(state);
			m_backLeft.setModuleState(state);
			m_backRight.setModuleState(state);
		}, null, this));
		m_gyro.zeroYaw();
		resetEncoders();
		// Wait 100 milliseconds to let all the encoders reset
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		m_odometry = new SwerveDriveOdometry(m_kinematics, getHeading(), getModulePositions());
	}

	/**
	 * Gets the robot's heading from the gyro.
	 * 
	 * @return The heading
	 */
	public Rotation2d getHeading() {
		if (RobotBase.isSimulation()) {
			return m_heading;
		}
		return m_gyro.getRotation2d();
	}

	/**
	 * Resets gyro heading to zero.
	 */
	public void resetHeading() {
		m_gyro.reset();
	}

	/**
	 * Resets drive encoders to zero.
	 */
	public void resetEncoders() {
		// Zero drive encoders
		m_frontLeft.resetDriveEncoder();
		m_frontRight.resetDriveEncoder();
		m_backLeft.resetDriveEncoder();
		m_backRight.resetDriveEncoder();
	}

	/**
	 * Returns robot pose.
	 * 
	 * @return The pose of the robot.
	 */
	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	/**
	 * 
	 * Stops all the motors.
	 */
	public void stopDriving() {
		drive(0, 0, 0, true);
	}

	/**
	 * Gets the module positions for each swerve module.
	 * 
	 * @return The module positions, in order of FL, FR, BL, BR
	 */
	public SwerveModulePosition[] getModulePositions() {
		return new SwerveModulePosition[] { m_frontLeft.getModulePosition(), m_frontRight.getModulePosition(),
				m_backLeft.getModulePosition(), m_backRight.getModulePosition() };
	}

	/**
	 * Drives the robot.
	 * 
	 * @param speeds          The chassis speeds.
	 * @param isFieldRelative Whether or not the chassis speeds are field-relative.
	 * @param isOpenLoop      Whether or not the modules will use open-loop control
	 *                        (no PID on drive motors).
	 */
	public void drive(ChassisSpeeds speeds, boolean isFieldRelative, boolean isOpenLoop) {
		if (isFieldRelative) {
			speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getHeading());
		}
		SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);
		if (isOpenLoop) {
			SwerveDriveKinematics.desaturateWheelSpeeds(states, kTeleopMaxVoltage);
		} else {
			SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxDrivetrainVelocity);
		}
		// Get the current module angles
		double[] moduleAngles = { m_frontLeft.getModuleAngle(), m_frontRight.getModuleAngle(),
				m_backLeft.getModuleAngle(), m_backRight.getModuleAngle() };
		for (int i = 0; i < states.length; i++) {
			// Optimize target module states
			states[i] = SwerveModuleState.optimize(states[i], Rotation2d.fromDegrees(moduleAngles[i]));
		}
		m_targetModuleStatePublisher.set(states);

		if (isOpenLoop) {
			m_frontLeft.setModuleState(states[0]);
			m_frontRight.setModuleState(states[1]);
			m_backLeft.setModuleState(states[2]);
			m_backRight.setModuleState(states[3]);
		} else {
			m_frontLeft.setModuleStateClosedLoop(states[0]);
			m_frontRight.setModuleStateClosedLoop(states[1]);
			m_backLeft.setModuleStateClosedLoop(states[2]);
			m_backRight.setModuleStateClosedLoop(states[3]);
		}
	}

	/**
	 * Drives the robot.
	 * 
	 * @param speedFwd        The forward speed in voltage
	 * @param speedSide       The sideways speed in voltage
	 * @param speedRot        The rotation speed in voltage
	 * @param isFieldRelative Whether or not the speeds are relative to the field
	 */
	public void drive(double speedFwd, double speedSide, double speedRot, boolean isFieldRelative) {
		drive(new ChassisSpeeds(speedFwd, speedSide, speedRot), isFieldRelative, true);
	}

	public void followTrajectory(Pose2d pose) {
		drive(new ChassisSpeeds(), true, false);
	}

	@Override
	public void periodic() {
		// SmartDashboard.putNumber("Current Position",
		// getModulePositions()[0].distanceMeters);
		SmartDashboard.putNumber("Heading Degrees", getHeading().getDegrees());
		SmartDashboard.putNumber("Heading Radians", getHeading().getRadians());
		m_posePublisher.set(m_odometry.update(getHeading(), getModulePositions()));
		SwerveModuleState[] states = { m_frontLeft.getModuleState(), m_frontRight.getModuleState(),
				m_backLeft.getModuleState(), m_backRight.getModuleState() };
		m_currentModuleStatePublisher.set(states);
	}

	/**
	 * Creates a command to drive the robot with joystick input.
	 *
	 * @return A command to drive the robot.
	 */
	public Command driveCommand(Supplier<Double> forwardSpeed, Supplier<Double> strafeSpeed,
			Supplier<Double> rotationRight, Supplier<Double> rotationLeft) {
		return run(() -> {
			// Get the forward, strafe, and rotation speed, using a deadband on the joystick
			// input so slight movements don't move the robot
			double rotSpeed = MathUtil.applyDeadband((rotationRight.get() - rotationLeft.get()),
					ControllerConstants.kDeadzone);
			rotSpeed = -Math.signum(rotSpeed) * Math.pow(rotSpeed, 2) * kTeleopMaxTurnVoltage;

			double fwdSpeed = MathUtil.applyDeadband(forwardSpeed.get(), ControllerConstants.kDeadzone);
			fwdSpeed = -Math.signum(fwdSpeed) * Math.pow(fwdSpeed, 2) * kTeleopMaxVoltage;

			double strSpeed = MathUtil.applyDeadband(strafeSpeed.get(), ControllerConstants.kDeadzone);
			strSpeed = -Math.signum(strSpeed) * Math.pow(strSpeed, 2) * kTeleopMaxVoltage;

			drive(fwdSpeed, strSpeed, rotSpeed, true);
		}).withName("DefaultDriveCommand");
	}

	/**
	 * Creates a command to drive the robot with joystick input in robot oriented
	 * controls.
	 * 
	 * @return A command to drive the robot.
	 */
	public Command robotOrientedDriveCommand(Supplier<Double> forwardSpeed, Supplier<Double> strafeSpeed,
			Supplier<Double> rotationRight, Supplier<Double> rotationLeft) {
		return run(() -> {
			// Get the forward, strafe, and rotation speed, using a deadband on the joystick
			// input so slight movements don't move the robot
			double rotSpeed = kTeleopMaxTurnVoltage * MathUtil.applyDeadband((rotationRight.get() - rotationLeft.get()),
					ControllerConstants.kDeadzone);
			rotSpeed = Math.signum(rotSpeed) * (rotSpeed * rotSpeed) * 12.0;
			double fwdSpeed = kTeleopMaxVoltage
					* MathUtil.applyDeadband(forwardSpeed.get(), ControllerConstants.kDeadzone);
			fwdSpeed = -Math.signum(fwdSpeed) * (fwdSpeed * fwdSpeed) * 12.0;
			double strSpeed = kTeleopMaxVoltage
					* MathUtil.applyDeadband(strafeSpeed.get(), ControllerConstants.kDeadzone);
			strSpeed = -Math.signum(strSpeed) * (strSpeed * strSpeed) * 12.0;
			drive(fwdSpeed, strSpeed, rotSpeed, false);
		}).withName("RobotOrientedDriveCommand");
	}

	/**
	 * Creates a command to reset the gyro heading to zero.
	 * 
	 * @return A command to reset the gyro heading.
	 */
	public Command resetHeadingCommand() {
		return runOnce(m_gyro::zeroYaw);
	}

	/**
	 * Creates a command to reset the drive encoders to zero.
	 * 
	 * @return A command to reset the drive encoders.
	 */
	public Command resetEncodersCommand() {
		return runOnce(() -> {
			resetEncoders();
			m_odometry.resetPosition(getHeading(), getModulePositions(), new Pose2d());
		});
	}

	/**
	 * Creates a command to align the swerve modules to zero degrees relative to the
	 * robot.
	 * 
	 * @return A command to align the swerve modules.
	 */
	public Command alignModulesToZeroComamnd() {
		return run(() -> {
			m_kinematics.resetHeadings(
					new Rotation2d[] { new Rotation2d(0), new Rotation2d(0), new Rotation2d(0), new Rotation2d(0) });
			drive(0, 0, 0, false);
		}).raceWith(Commands.waitSeconds(5));
	}

	/**
	 * Creates a command to run a SysId quasistatic test.
	 * 
	 * @param direction The direction to run the test in.
	 * @return The command.
	 */
	public Command sysidQuasistatic(SysIdRoutine.Direction direction) {
		return m_sysidRoutine.quasistatic(direction);
	}

	/**
	 * Creates a command to run a SysId dynamic test.
	 * 
	 * @param direction The direction to run the test in.
	 * @return The command.
	 */
	public Command sysidDynamic(SysIdRoutine.Direction direction) {
		return m_sysidRoutine.dynamic(direction);
	}
}
