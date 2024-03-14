// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.ProtobufPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
	private double m_headingOffset = 0;
	private Pose2d m_pose = new Pose2d(0, 0, new Rotation2d());
	private Rotation2d m_heading = new Rotation2d(Math.PI / 2);
	private final Field2d m_field = new Field2d();

	private final ProtobufPublisher<Pose2d> m_posePublisher;
	private final StructArrayPublisher<SwerveModuleState> m_targetModuleStatePublisher;
	private final StructArrayPublisher<SwerveModuleState> m_currentModuleStatePublisher;

	/** Creates a new DriveSubsystem. */
	public DriveSubsystem() {
		SmartDashboard.putData("Field", m_field);
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
	 * Sets a gyro offset.
	 * 
	 * @param angle The angle to offset gyro readings (CCW+).
	 */
	public void setHeadingOffset(double angle) {
		m_headingOffset = angle;
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
		return Rotation2d.fromDegrees(-m_gyro.getYaw() + m_headingOffset);
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

	public void setIdleMode(IdleMode mode) {
		m_frontLeft.setIdleMode(mode);
		m_frontRight.setIdleMode(mode);
		m_backLeft.setIdleMode(mode);
		m_backRight.setIdleMode(mode);
	}

	/**
	 * Returns robot pose.
	 * 
	 * @return The pose of the robot.
	 */
	public Pose2d getPose() {
		return m_pose;
	}

	/**
	 * Calculates the modules states needed for the robot to achieve the target
	 * chassis speed.
	 * 
	 * @param speeds          The target chassis speed
	 * @param isFieldRelative Whether or not the chassis speeds are field-relative
	 * @return The module states, in order of FL, FR, BL, BR
	 */
	public SwerveModuleState[] calculateModuleStates(ChassisSpeeds speeds, boolean isFieldRelative) {
		if (isFieldRelative) {
			speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getHeading());
		}
		if (RobotBase.isSimulation()) {
			updateSimPose(speeds);
		}
		SmartDashboard.putNumber("Heading Radians", getHeading().getRadians());
		SmartDashboard.putNumber("Heading Degrees", getHeading().getDegrees());

		SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxSpeed);
		m_field.setRobotPose(m_pose);
		return states;
	}

	/**
	 * Updates the sim pose.
	 * 
	 * @param speeds The chassis speeds to use
	 */
	private void updateSimPose(ChassisSpeeds speeds) {
		var transform = new Transform2d(speeds.vxMetersPerSecond * kModuleResponseTimeSeconds * 3,
				speeds.vyMetersPerSecond * kModuleResponseTimeSeconds * 3, new Rotation2d(
						speeds.omegaRadiansPerSecond * kModuleResponseTimeSeconds * 3));

		m_pose = m_pose.plus(transform);
		m_heading = m_pose.getRotation();
		m_posePublisher.set(m_pose);
	}

	/**
	 * 
	 * Stops all the motors.
	 */
	public void stopDriving() {
		setModuleStates(calculateModuleStates(new ChassisSpeeds(0, 0, 0), true));
	}

	public void setRampRate(double rampRate) {
		m_backLeft.setRampRate(rampRate);
		m_backRight.setRampRate(rampRate);
		m_frontLeft.setRampRate(rampRate);
		m_frontRight.setRampRate(rampRate);
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
	 * Sets module states for each swerve module.
	 * 
	 * @param moduleStates The module states, in order of FL, FR, BL, BR
	 */
	public void setModuleStates(SwerveModuleState[] moduleStates) {
		// Get the current module angles
		double[] moduleAngles = { m_frontLeft.getModuleAngle(), m_frontRight.getModuleAngle(),
				m_backLeft.getModuleAngle(), m_backRight.getModuleAngle() };
		for (int i = 0; i < moduleStates.length; i++) {
			// Optimize target module states
			moduleStates[i] = SwerveModuleState.optimize(moduleStates[i], Rotation2d.fromDegrees(moduleAngles[i]));
		}
		m_targetModuleStatePublisher.set(moduleStates);

		m_frontLeft.setModuleState(moduleStates[0]);
		m_frontRight.setModuleState(moduleStates[1]);
		m_backLeft.setModuleState(moduleStates[2]);
		m_backRight.setModuleState(moduleStates[3]);
	}

	/**
	 * Directly sets the module angle. Do not use for general driving.
	 * 
	 * @param angle The angle in degrees
	 */
	public void setModuleAngles(double angle) {
		m_frontLeft.setAngle(angle);
		m_frontRight.setAngle(angle);
		m_backLeft.setAngle(angle);
		m_backRight.setAngle(angle);
	}

	public void setModuleStatesDirect(SwerveModuleState moduleState) {
		m_frontLeft.setModuleState(moduleState);
		m_frontRight.setModuleState(moduleState);
		m_backLeft.setModuleState(moduleState);
		m_backRight.setModuleState(moduleState);
	}

	public void setModuleSpeeds(double speed) {
		m_frontLeft.setSpeed(speed);
		m_frontRight.setSpeed(speed);
		m_backLeft.setSpeed(speed);
		m_backRight.setSpeed(speed);
	}

	/**
	 * Sets module states for each swerve module.
	 * 
	 * @param speedFwd        The forward speed
	 * @param speedSide       The sideways speed
	 * @param speedRot        The rotation speed
	 * @param isFieldRelative Whether or not the speeds are relative to the field
	 */
	public void setModuleStates(double speedFwd, double speedSide, double speedRot, boolean isFieldRelative) {
		setModuleStates(calculateModuleStates(new ChassisSpeeds(speedFwd, speedSide, speedRot), isFieldRelative));
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Current Position", getModulePositions()[0].distanceMeters);
		SmartDashboard.putNumber("Heading Degrees", getHeading().getDegrees());
		if (RobotBase.isReal()) {
			m_pose = m_odometry.update(getHeading(), getModulePositions());
			m_posePublisher.set(m_pose);
		}
		SwerveModuleState[] states = { m_frontLeft.getModuleState(), m_frontRight.getModuleState(),
				m_backLeft.getModuleState(), m_backRight.getModuleState() };
		m_currentModuleStatePublisher.set(states);
		SmartDashboard.putNumber("Drive FR motor temperature", m_frontRight.getDriveTemperature());
		SmartDashboard.putNumber("Drive BR motor temperature", m_backRight.getDriveTemperature());
		SmartDashboard.putNumber("Drive BL motor temperature", m_backLeft.getDriveTemperature());
		SmartDashboard.putNumber("Drive FL motor temperature", m_frontLeft.getDriveTemperature());
		SmartDashboard.putNumber("Drive FR steer current", m_frontRight.getSteerCurrent());
		SmartDashboard.putNumber("Drive BR steer current", m_backRight.getSteerCurrent());
		SmartDashboard.putNumber("Drive BL steer current", m_backLeft.getSteerCurrent());
		SmartDashboard.putNumber("Drive FL steer current", m_frontLeft.getSteerCurrent());
		SmartDashboard.putNumber("Back Right Current", m_backRight.getDriveCurrent());
		SmartDashboard.putNumber("Back Left Current", m_backLeft.getDriveCurrent());
		SmartDashboard.putNumber("Front Right Current", m_frontRight.getDriveCurrent());
		SmartDashboard.putNumber("Front Left Current", m_frontLeft.getDriveCurrent());
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
			double rotSpeed = kTeleopMaxTurnSpeed * MathUtil.applyDeadband((rotationRight.get() - rotationLeft.get()),
					ControllerConstants.kDeadzone);
			rotSpeed = Math.signum(rotSpeed) * (rotSpeed * rotSpeed);
			double fwdSpeed = kTeleopMaxSpeed
					* MathUtil.applyDeadband(forwardSpeed.get(), ControllerConstants.kDeadzone);
			fwdSpeed = Math.signum(fwdSpeed) * (fwdSpeed * fwdSpeed);
			double strSpeed = kTeleopMaxSpeed
					* MathUtil.applyDeadband(strafeSpeed.get(), ControllerConstants.kDeadzone);
			strSpeed = Math.signum(strSpeed) * (strSpeed * strSpeed);
			setModuleStates(calculateModuleStates(new ChassisSpeeds(fwdSpeed, strSpeed, rotSpeed), true));
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
			double rotSpeed = kTeleopMaxTurnSpeed * MathUtil.applyDeadband((rotationRight.get() - rotationLeft.get()),
					ControllerConstants.kDeadzone);
			rotSpeed = Math.signum(rotSpeed) * (rotSpeed * rotSpeed);
			double fwdSpeed = kTeleopMaxSpeed
					* MathUtil.applyDeadband(forwardSpeed.get(), ControllerConstants.kDeadzone);
			fwdSpeed = Math.signum(fwdSpeed) * (fwdSpeed * fwdSpeed);
			double strSpeed = kTeleopMaxSpeed
					* MathUtil.applyDeadband(strafeSpeed.get(), ControllerConstants.kDeadzone);
			strSpeed = Math.signum(strSpeed) * (strSpeed * strSpeed);
			setModuleStates(calculateModuleStates(new ChassisSpeeds(fwdSpeed, strSpeed, rotSpeed), false));
			if (fwdSpeed > 0.2 || rotSpeed > 0.2 || strSpeed > 0.2) {
				setRampRate(.3);
			} else {
				setRampRate(0.1);
			}
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

	public Command offsetGyroCommand(double angle) {
		return runOnce(() -> setHeadingOffset(angle));
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
			setModuleStates(0, 0, 0, false);
		}).raceWith(Commands.waitSeconds(5));
	}
}
