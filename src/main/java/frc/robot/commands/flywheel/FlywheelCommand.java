// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;

public class FlywheelCommand extends Command {
	private final FlywheelOperation m_operation;
	private final FlywheelSubsystem m_flywheelSubsystem;
	private final double m_topRPM;
	private final double m_bottomRPM;

	public enum FlywheelOperation {
		/** Set the velocity and end immediately. */
		SET_VELOCITY,
		/** Set the velocity and end when at setpoint. */
		SETTLE,
		/** Reverses the flywheel */
		REVERSE;
	}

	/**
	 * Creates a new FlywheelCommand.
	 * 
	 * @param flywheelSubsytem The subsystem.
	 * @param operation        The operation.
	 * @param rpm              The RPM to spin the motor at.
	 */
	public FlywheelCommand(FlywheelSubsystem flywheelSubsytem, FlywheelOperation operation, double top_rpm,
			double bottom_rpm) {
		m_flywheelSubsystem = flywheelSubsytem;
		m_topRPM = top_rpm;
		m_bottomRPM = bottom_rpm;
		m_operation = operation;
		addRequirements(m_flywheelSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		if (m_operation == FlywheelOperation.SET_VELOCITY) {
			m_flywheelSubsystem.setBottomVelocity(m_bottomRPM);
			m_flywheelSubsystem.setTopVelocity(m_topRPM);
		}
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		if (m_operation == FlywheelOperation.SET_VELOCITY) {
			return true;
		} else if (m_operation == FlywheelOperation.SETTLE) {
			return m_flywheelSubsystem.atSetpoint();
		}
		System.out.println("Unreachable code in FlywheelCommand");
		return false; // unreachable code
	}
}
