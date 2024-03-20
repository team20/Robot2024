// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import java.io.File;

import javax.sound.sampled.AudioSystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerShootCommand extends Command {
	private IndexerSubsystem m_indexerSubsystem;
	private Timer m_timer;
	private double m_duration;

	/** Creates a new IndexerShootCommand. */
	public IndexerShootCommand(IndexerSubsystem indexerSubsystem) {
		this(Constants.IndexerConstants.kKickTime, indexerSubsystem);
	}

	public IndexerShootCommand(double duration, IndexerSubsystem indexerSubsystem) {
		m_timer = new Timer();
		m_duration = duration;
		m_indexerSubsystem = indexerSubsystem;
		if (m_indexerSubsystem != null)
			addRequirements(indexerSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_timer.reset();
		m_timer.start();
		if (m_indexerSubsystem != null)
			m_indexerSubsystem.setSpeed(Constants.IndexerConstants.kKickSpeed);
		new Thread(() -> {
			try {
				var clip = AudioSystem.getClip();
				clip.open(AudioSystem
						.getAudioInputStream(
								new File("." + File.separator + "src" + File.separator + "main" + File.separator
										+ "deploy" + File.separator + "shot.wav")));
				clip.start();
			} catch (Exception e) {
			}
		}).start();
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		if (m_indexerSubsystem != null)
			m_indexerSubsystem.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return m_timer.hasElapsed(m_duration);
	}

}
