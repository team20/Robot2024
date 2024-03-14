// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ClampedController;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToAngleCommand extends Command {
	private final double m_targetAngle;
	private final double m_angleThreshold;
	private final DriveSubsystem m_driveSubsystem;
	private final ClampedController m_controller = new ClampedController(0.01, 0.1, .5);
	private final boolean m_relative;

	/**
	 * Creates a command to turn the robot to an angle.
	 * 
	 * @param driveSubsystem The drive subsystem.
	 * @param targetAngle    The target angle (CCW+).
	 * @param angleThreshold The tolerance of the angle.
	 * @param relative       Whether or not the angle is relative to the current
	 *                       heading.
	 */
	public TurnToAngleCommand(DriveSubsystem driveSubsystem, double targetAngle, double angleThreshold,
			boolean relative) {
		m_targetAngle = targetAngle;
		m_angleThreshold = angleThreshold;
		m_driveSubsystem = driveSubsystem;
		m_relative = relative;
		m_controller.setTolerance(m_angleThreshold);
		m_controller.enableContinuousInput(0, 360);
		addRequirements(driveSubsystem);
	}

	/**
	 * Creates a command to turn the robot to an angle.
	 * 
	 * @param driveSubsystem The drive subsystem.
	 * @param targetAngle    The target angle (CCW+).
	 * @param relative       Whether or not the angle is relative to the current
	 *                       heading.
	 */
	public TurnToAngleCommand(DriveSubsystem driveSubsystem, double targetAngle, boolean relative) {
		this(driveSubsystem, targetAngle, 3, relative);
	}

	@Override
	public void initialize() {
		if (m_relative) {
			m_controller.setSetpoint(m_driveSubsystem.getHeading().getDegrees() + m_targetAngle);
		} else {
			m_controller.setSetpoint(m_targetAngle);
		}
	}

	@Override
	public void execute() {
		double speed = -m_controller.calculate(m_driveSubsystem.getHeading().getDegrees());
		m_driveSubsystem.setModuleStates(0, 0, speed, false);
	}

	@Override
	public boolean isFinished() {
		return m_controller.atSetpoint();
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_driveSubsystem.stopDriving();
	}
}
