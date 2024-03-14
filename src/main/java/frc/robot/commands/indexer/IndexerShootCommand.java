// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerShootCommand extends Command {
	private IndexerSubsystem m_indexerSubsystem;
	private Timer m_timer;

	/** Creates a new IndexerShootCommand. */
	public IndexerShootCommand(IndexerSubsystem indexerSubsystem) {
		m_timer = new Timer();
		m_indexerSubsystem = indexerSubsystem;
		addRequirements(indexerSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_timer.reset();
		m_timer.start();
		m_indexerSubsystem.setSpeed(Constants.IndexerConstants.kKickSpeed);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_indexerSubsystem.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return m_timer.hasElapsed(Constants.IndexerConstants.kKickTime);
	}
}
