// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.DriveConstants.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Contains all the hardware and controllers for a swerve module.
 */
public class SwerveModule {
	private final PIDController m_PIDController = new PIDController(0.0030, 0, 0);
	private final CANcoder m_CANCoder;
	private final TalonFX m_driveMotor;
	private final CANSparkMax m_steerMotor;
	private final DCMotorSim m_driveMotorModel = new DCMotorSim(DCMotor.getKrakenX60(1), 1, 0.0005);
	private final DCMotorSim m_steerMotorModel = new DCMotorSim(DCMotor.getNEO(1), 1, 0.00025);

	public SwerveModule(int CANport, int drivePort, int steerPort) {
		m_CANCoder = new CANcoder(CANport);
		m_driveMotor = new TalonFX(drivePort);
		m_steerMotor = new CANSparkMax(steerPort, MotorType.kBrushless);
		m_PIDController.setIZone(kIz);
		m_driveMotor.getConfigurator().apply(kDriveConfig);
		configMotorController(m_steerMotor, kSteerSmartCurrentLimit, kSteerPeakCurrentLimit);
		m_PIDController.enableContinuousInput(0, 360);
	}

	/**
	 * Configures our motors with the exact same settings
	 * 
	 * @param motorController The CANSparkMax to configure
	 */
	private void configMotorController(CANSparkMax motorController, int smartCurrentLimit, int peakCurrentLimit) {
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
		return m_driveMotor.getPosition().getValueAsDouble() * kMotorRotationsPerMeter;
	}

	public double getSteerCurrent() {
		return m_steerMotor.getOutputCurrent();
	}

	public double getDriveCurrent() {
		return m_driveMotor.getSupplyCurrent().getValueAsDouble();
	}

	/**
	 * Resets drive encoder to zero.
	 */
	public void resetDriveEncoder() {
		m_driveMotor.setPosition(0);
	}

	/**
	 * Gets the current drive motor voltage.
	 * 
	 * @return The motor speed in voltage
	 */
	public double getDriveVoltage() {
		return m_driveMotor.getMotorVoltage().getValueAsDouble();
	}

	/**
	 * Gets the current drive motor temperature.
	 * 
	 * @return The temperature in degrees celcius
	 */
	public double getDriveTemperature() {
		return m_driveMotor.getDeviceTemp().getValueAsDouble();
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
		return new SwerveModuleState(getDriveVoltage(), Rotation2d.fromDegrees(getModuleAngle()));
	}

	/**
	 * Sets the drive motor speeds and module angle.
	 * 
	 * @param state The module state
	 */
	public void setModuleState(SwerveModuleState state) {
		double power = state.speedMetersPerSecond;
		double currVoltage = RobotController.getBatteryVoltage();
		if (currVoltage < 7) {
			power *= 0.1;
		} else if (currVoltage < 8) {
			power *= 0.2;
		} else if (currVoltage < 9) {
			power *= 0.4;
		} else if (currVoltage < 10) {
			power *= 0.8;
		}
		m_driveMotor.setVoltage(power);
		double turnPower = m_PIDController.calculate(getModuleAngle(), state.angle.getDegrees());
		m_steerMotor.setVoltage(turnPower);
		if (RobotBase.isSimulation()) {
			var driveMotorSimState = m_driveMotor.getSimState();
			m_driveMotorModel.setInputVoltage(driveMotorSimState.getMotorVoltage());
			m_driveMotorModel.update(0.02);
			driveMotorSimState.setRawRotorPosition(m_driveMotorModel.getAngularPositionRotations());
			driveMotorSimState.setRotorVelocity(m_driveMotorModel.getAngularVelocityRPM() / 60.0);
			var encoderSimState = m_CANCoder.getSimState();
			m_steerMotorModel.setInputVoltage(turnPower);
			m_steerMotorModel.update(0.02);
			encoderSimState.setRawPosition(m_steerMotorModel.getAngularPositionRotations());
			encoderSimState.setVelocity(m_steerMotorModel.getAngularVelocityRPM());
		}
	}

	/**
	 * Sets the module angle.
	 * 
	 * @param angle
	 */
	public void setAngle(double angle) {
		var out = m_PIDController.calculate(getModuleAngle(), angle);
		SmartDashboard.putNumber("PID out" + m_driveMotor.getDeviceID(), out);
		m_steerMotor.setVoltage(out);
	}
}
