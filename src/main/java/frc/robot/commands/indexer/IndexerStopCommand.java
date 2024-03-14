// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerStopCommand extends Command {
	private IndexerSubsystem m_indexerSubsystem;

	/** Creates a new IndexerStopCommand. */
	public IndexerStopCommand(IndexerSubsystem indexerSubsystem) {
		m_indexerSubsystem = indexerSubsystem;
		addRequirements(indexerSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_indexerSubsystem.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return true;
	}
}
