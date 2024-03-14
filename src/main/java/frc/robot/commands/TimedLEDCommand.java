// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.ArduinoSubsystem.StatusCode;

public class TimedLEDCommand extends Command {
	private final ArduinoSubsystem m_arduinoSubsystem;
	private final double m_seconds;
	private final Timer m_timer = new Timer();
	private final StatusCode m_statusCode;

	/**
	 * Creates a new TimedLEDCommand.
	 * 
	 * @param subsystem  The subsystem
	 * @param seconds    Time to drive in seconds
	 * @param statusCode The LED pattern to display
	 */
	public TimedLEDCommand(ArduinoSubsystem subsystem, double seconds, StatusCode statusCode) {
		m_arduinoSubsystem = subsystem;
		m_seconds = seconds;
		m_statusCode = statusCode;
		addRequirements(subsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_timer.restart();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_arduinoSubsystem.setCode(m_statusCode);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_arduinoSubsystem.setCode(StatusCode.DEFAULT);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return m_timer.get() >= m_seconds;
	}
}
