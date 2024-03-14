// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ClampedController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SimpleVisionSubsystem;

public class SimpleVisionAlignCommand extends Command {
	private final DriveSubsystem m_driveSubsystem;
	private final SimpleVisionSubsystem m_visionSubsystem;
	private final ClampedController m_controller = new ClampedController(0.01, 0.1, 1);

	/**
	 * Creates a new SimpleVisionAlignCommand.
	 * 
	 * @param driveSubsystem  The drive subsystem.
	 * @param visionSubsystem The vision subsystem.
	 */
	public SimpleVisionAlignCommand(DriveSubsystem driveSubsystem, SimpleVisionSubsystem visionSubsystem) {
		m_driveSubsystem = driveSubsystem;
		m_visionSubsystem = visionSubsystem;
		m_controller.enableContinuousInput(0, 360);
		addRequirements(m_driveSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		// m_controller.setSetpoint(0);

	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// Get angle to tag from VisionSubsystem
		double currentAngle = m_visionSubsystem.getAngle();
		// Set power using PIDController
		double power = m_controller.calculate(currentAngle);
		// Turn to tag using PID power
		m_driveSubsystem.setModuleStates(0, 0, -power, false);
		// SmartDashboard.putNumber("power", power);
		// SmartDashboard.putNumber("currentAngle", currentAngle);

		// double currentAngle = m_visionSubsystem.getAngle();
		// CommandComposer.TurnToAngleCommand(m_driveSubsystem, currentAngle, 2, true);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_driveSubsystem.stopDriving();
		// SmartDashboard.putNumber("power", 0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
		// (Math.abs(m_visionSubsystem.getAngle()) < m_tolerance); // ||
		// m_timer.get() > 2);
	}
}
