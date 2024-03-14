// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import static frc.robot.Constants.IndexerConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerCommand extends Command {
	private final double m_speed;
	private final IndexerSubsystem m_indexerSubsystem;

	public static IndexerCommand getFowardCommand(IndexerSubsystem indexerSubsystem) {
		return new IndexerCommand(indexerSubsystem, kIntakeSpeed);
	}

	public static IndexerCommand getReverseCommand(IndexerSubsystem indexerSubsystem) {
		return new IndexerCommand(indexerSubsystem, kReverseSpeed);
	}

	/** Creates a new IndexerFowardCommand. */
	private IndexerCommand(IndexerSubsystem indexerSubsystem, double speed) {
		m_speed = speed;
		m_indexerSubsystem = indexerSubsystem;
		addRequirements(m_indexerSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_indexerSubsystem.setSpeed(m_speed);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return true;
	}
}
