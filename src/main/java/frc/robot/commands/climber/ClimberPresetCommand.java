// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberPresetCommand extends Command {
	private final ClimberSubsystem m_climberSubsystem;

	/** Creates a new ClimberMove. */
	public ClimberPresetCommand(ClimberSubsystem subsystem) {
		m_climberSubsystem = subsystem;
		addRequirements(m_climberSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		System.out.println("command ran");
		if (m_climberSubsystem.atZero()) {
			m_climberSubsystem.setPosition(ClimbConstants.kMaxExtension, ClimbConstants.kMaxExtension);
			System.out.println("up");
		} else {
			m_climberSubsystem.setPosition(0, 0);
			System.out.println("down");
		}
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return m_climberSubsystem.atleftSetpoint() && m_climberSubsystem.atrightSetpoint();
	}
}