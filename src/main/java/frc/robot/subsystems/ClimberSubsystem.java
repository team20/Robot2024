// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimberSubsystem extends SubsystemBase {

	private final CANSparkMax m_leftMotor = new CANSparkMax(ClimbConstants.kLeftPort, MotorType.kBrushless);
	private final CANSparkMax m_rightMotor = new CANSparkMax(ClimbConstants.kRightPort, MotorType.kBrushless);

	private final RelativeEncoder m_leftEncoder = m_leftMotor.getEncoder();
	private final SparkPIDController m_leftPidController = m_leftMotor.getPIDController();

	private final RelativeEncoder m_rightEncoder = m_rightMotor.getEncoder();
	private final SparkPIDController m_rightPidController = m_rightMotor.getPIDController();

	private double m_setPositionLeft = 0;
	private double m_setPositionRight = 0;

	private double m_setSpeedLeft = 0;
	private double m_setSpeedRight = 0;

	/** Creates a new ClimberSubsystem. */
	public ClimberSubsystem() {

		m_leftMotor.restoreFactoryDefaults();
		m_leftMotor.setInverted(ClimbConstants.kLeftInvert);
		m_leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		m_leftMotor.enableVoltageCompensation(12);
		m_leftMotor.setSmartCurrentLimit(ClimbConstants.kSmartCurrentLimit);
		m_leftMotor.setSecondaryCurrentLimit(ClimbConstants.kSecondaryCurrentLimit);
		m_leftMotor.setSoftLimit(SoftLimitDirection.kForward, ClimbConstants.kMaxExtension);
		m_leftMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
		m_leftMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
		m_leftMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

		m_rightMotor.restoreFactoryDefaults();
		m_rightMotor.setInverted(ClimbConstants.kRightInvert);
		m_rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		m_rightMotor.enableVoltageCompensation(12);
		m_rightMotor.setSmartCurrentLimit(ClimbConstants.kSmartCurrentLimit);
		m_rightMotor.setSecondaryCurrentLimit(ClimbConstants.kSecondaryCurrentLimit);
		m_rightMotor.setSoftLimit(SoftLimitDirection.kForward, ClimbConstants.kMaxExtension);
		m_rightMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
		m_rightMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
		m_rightMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

		m_leftPidController.setP(ClimbConstants.kP);
		m_leftPidController.setI(ClimbConstants.kI);
		m_leftPidController.setD(ClimbConstants.kD);
		m_leftPidController.setFF(ClimbConstants.kFF);
		m_leftPidController.setOutputRange(ClimbConstants.kMinOutput, ClimbConstants.kMaxOutput);

		m_rightPidController.setP(ClimbConstants.kP);
		m_rightPidController.setI(ClimbConstants.kI);
		m_rightPidController.setD(ClimbConstants.kD);
		m_rightPidController.setFF(ClimbConstants.kFF);
		m_rightPidController.setOutputRange(ClimbConstants.kMinOutput, ClimbConstants.kMaxOutput);
		resetEncoder();
	}

	@Override
	public void periodic() {
		// SmartDashboard.putNumber("Left Climber Position", getleftPosition());
		// SmartDashboard.putNumber("Right Climber Position", getrightPosition());
	}

	// returns the position of the left or right motor
	public double getleftPosition() {
		return m_leftEncoder.getPosition();
	}

	public double getrightPosition() {
		return m_rightEncoder.getPosition();
	}

	// returns true if the motor is at the setpoint
	public boolean atleftSetpoint() {
		return (Math.abs(m_setPositionLeft - getleftPosition()) <= ClimbConstants.kTolerance);
	}

	public boolean atrightSetpoint() {
		return (Math.abs(m_setPositionRight - getrightPosition()) <= ClimbConstants.kTolerance);
	}

	public void setPosition(double positionLeft, double positionRight) {
		m_setPositionLeft = positionLeft;
		m_setPositionRight = positionRight;
		m_leftPidController.setReference(positionLeft, ControlType.kPosition);
		m_rightPidController.setReference(positionRight, ControlType.kPosition);
	}

	public void setSpeed(double speedLeft, double speedRight) {
		m_setSpeedLeft = speedLeft;
		m_setSpeedRight = speedRight;
		m_leftMotor.set(m_setSpeedLeft);
		m_rightMotor.set(m_setSpeedRight);

	}

	public void resetEncoder() {
		m_leftEncoder.setPosition(0);
		m_rightEncoder.setPosition(0);
		setPosition(0, 0);
	}

	public double getOutputCurrent() {
		return Math.max(Math.abs(m_leftMotor.getOutputCurrent()), Math.abs(m_rightMotor.getOutputCurrent()));
	}

	public boolean atZero() {
		return ((Math.abs(getrightPosition()) <= ClimbConstants.kTolerance)
				&& (Math.abs(getleftPosition()) <= ClimbConstants.kTolerance));
	}
}