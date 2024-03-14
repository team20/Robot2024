// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.aimshooter;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.AimerSubsystem;

public class AimerDriveCommand extends Command {

	private final AimerSubsystem m_aimerSubsystem;
	private final Supplier<Double> m_speed;

	/** Creates a new AimerDriveCommand. */
	public AimerDriveCommand(AimerSubsystem subsystem, Supplier<Double> speed) {
		m_aimerSubsystem = subsystem;
		m_speed = speed;
		addRequirements(m_aimerSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		double speed = MathUtil.applyDeadband(m_speed.get(), ControllerConstants.kDeadzone) * -1;
		if (speed != 0) {
			m_aimerSubsystem.setManual(true); // Joystick control
			// Prevent lead screw from going out of bounds
			if ((m_aimerSubsystem.getAimerHeight() > 1) && (speed > 0)) {
				speed = 0;
			} else if ((m_aimerSubsystem.getAimerHeight() <= 0.05) && (speed < 0)) {
				speed = 0;
			}
			m_aimerSubsystem.setSpeed(speed);
		} else {
			m_aimerSubsystem.setManual(false); // Setpoint control
			m_aimerSubsystem.setAimerHeight(m_aimerSubsystem.getAimerHeight());
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_aimerSubsystem.stopMotor();
	}
}