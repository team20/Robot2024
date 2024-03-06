// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
	private CANSparkMax m_indexerMotor = new CANSparkMax(IndexerConstants.kIndexerPort, MotorType.kBrushless);;
	private final RelativeEncoder m_encoder = m_indexerMotor.getEncoder();
	private DigitalInput m_proximitySensor;

	/**
	 * Creates a new IndexerSubsystem.
	 * 
	 * 
	 */
	public IndexerSubsystem() {
		m_indexerMotor.setIdleMode(IdleMode.kBrake);
		m_indexerMotor.setInverted(IndexerConstants.kInvert);
		m_indexerMotor.enableVoltageCompensation(12);
		m_indexerMotor.setSmartCurrentLimit(IndexerConstants.kIndexerSmartCurrentLimit);
		m_indexerMotor.setSecondaryCurrentLimit(IndexerConstants.kIndexerPeakCurrentLimit);
		m_proximitySensor = new DigitalInput(1);
	}

	public void setSpeed(double speed) {
		m_indexerMotor.set(speed);
	}

	public void stop() {
		setSpeed(0.0);
	}

	public boolean getLimitSwitch() {
		return !m_proximitySensor.get();
	}

	public void periodic() {
		SmartDashboard.putNumber("Indexer Current", m_indexerMotor.getOutputCurrent());
		SmartDashboard.putNumber("Indexer Velocity", m_encoder.getVelocity());
		SmartDashboard.putBoolean("Limit Switch Value", m_proximitySensor.get());
	}
}
