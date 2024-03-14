// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.DriveConstants.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Contains all the hardware and controllers for a swerve module.
 */
public class SwerveModule {
	private final PIDController m_PIDController = new PIDController(kP, kI, kD);
	private final CANcoder m_CANCoder;
	private final CANSparkMax m_driveMotor;
	private final RelativeEncoder m_driveEncoder;
	private final CANSparkMax m_steerMotor;

	public SwerveModule(int CANport, int drivePort, int steerPort) {
		m_CANCoder = new CANcoder(CANport);
		m_driveMotor = new CANSparkMax(drivePort, MotorType.kBrushless);
		m_steerMotor = new CANSparkMax(steerPort, MotorType.kBrushless);
		m_PIDController.setIZone(kIz);
		m_driveEncoder = m_driveMotor.getEncoder();
		configMotorController(m_driveMotor, kDrivePeakCurrentLimit);
		configMotorSteerController(m_steerMotor, kSteerSmartCurrentLimit, kSteerPeakCurrentLimit);
		m_PIDController.enableContinuousInput(0, 360);
		m_driveMotor.setOpenLoopRampRate(kRampRate);
		m_driveMotor.setClosedLoopRampRate(kRampRate);
	}

	/**
	 * Configures our motors with the exact same settings
	 * 
	 * @param motorController The CANSparkMax to configure
	 */
	private void configMotorController(CANSparkMax motorController, int peakCurrentLimit) {
		motorController.restoreFactoryDefaults();
		motorController.setIdleMode(IdleMode.kBrake);
		motorController.enableVoltageCompensation(12);
		// motorController.setSmartCurrentLimit(smartCurrentLimit);
		motorController.setSecondaryCurrentLimit(peakCurrentLimit);
	}

	/**
	 * Configures our motors with the exact same settings
	 * 
	 * @param motorController The CANSparkMax to configure
	 */
	private void configMotorSteerController(CANSparkMax motorController, int smartCurrentLimit, int peakCurrentLimit) {
		motorController.restoreFactoryDefaults();
		motorController.setIdleMode(IdleMode.kBrake);
		motorController.enableVoltageCompensation(12);
		motorController.setSmartCurrentLimit(smartCurrentLimit);
		motorController.setSecondaryCurrentLimit(peakCurrentLimit);
	}

	/**
	 * Returns drive encoder distance in meters traveled.
	 * 
	 * @return The position
	 */
	public double getDriveEncoderPosition() {
		return this.m_driveEncoder.getPosition() * kMotorRotationsPerMeter;
	}

	public double getSteerCurrent() {
		return m_steerMotor.getOutputCurrent();
	}

	public double getDriveCurrent() {
		return m_driveMotor.getOutputCurrent();
	}

	/**
	 * Resets drive encoder to zero.
	 */
	public void resetDriveEncoder() {
		m_driveEncoder.setPosition(0);
	}

	/**
	 * Gets the current drive motor speed.
	 * 
	 * @return The motor speed in percent output
	 */
	public double getDriveSpeed() {
		return m_driveMotor.getAppliedOutput();
	}

	/**
	 * Gets the current drive motor temperature.
	 * 
	 * @return The temperature in degrees celcius
	 */
	public double getDriveTemperature() {
		return m_driveMotor.getMotorTemperature();
	}

	/**
	 * Returns the module angle in degrees.
	 * 
	 * @return The module angle
	 */
	public double getModuleAngle() {
		return m_CANCoder.getAbsolutePosition().getValueAsDouble() * 360;
	}

	/**
	 * Sets the module angle in degrees.
	 * 
	 * @param angle The angle in degrees
	 */
	public void setModuleAngle(double angle) {
		m_PIDController.setSetpoint(angle);
	}

	/**
	 * Returns the module position.
	 * 
	 * @return The module position
	 */
	public SwerveModulePosition getModulePosition() {
		return new SwerveModulePosition(getDriveEncoderPosition(), Rotation2d.fromDegrees(getModuleAngle()));
	}

	/**
	 * Gets the module speed and angle.
	 * 
	 * @return The module state
	 */
	public SwerveModuleState getModuleState() {
		return new SwerveModuleState(m_driveMotor.getAppliedOutput(), Rotation2d.fromDegrees(getModuleAngle()));
	}

	/**
	 * Sets the drive motor speeds and module angle.
	 * 
	 * @param state The module state
	 */
	public void setModuleState(SwerveModuleState state) {
		m_driveMotor.set(state.speedMetersPerSecond);
		m_steerMotor.set(m_PIDController.calculate(getModuleAngle(), state.angle.getDegrees()));
	}

	/**
	 * Sets the module angle.
	 * 
	 * @param angle
	 */
	public void setAngle(double angle) {
		var out = m_PIDController.calculate(getModuleAngle(), angle);
		SmartDashboard.putNumber("PID out" + m_driveMotor.getDeviceId(), out);
		m_steerMotor.set(out);
	}

	public void setSpeed(double speed) {
		m_driveMotor.set(speed);
	}

	public void setIdleMode(IdleMode mode) {
		m_driveMotor.setIdleMode(mode);
		m_steerMotor.setIdleMode(mode);
	}

	public void setRampRate(double rampRate) {
		m_driveMotor.setOpenLoopRampRate(rampRate);
		m_driveMotor.setClosedLoopRampRate(rampRate);
	}

}
