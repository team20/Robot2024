// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberDriveCommand extends Command {

	private final ClimberSubsystem m_climberSubsystem;
	private final Supplier<Double> m_left;
	private final Supplier<Double> m_right;
	private double leftSpeed;
	private double rightSpeed;

	/** Creates a new LeftClimberDrive. */
	public ClimberDriveCommand(ClimberSubsystem subsystem, Supplier<Double> left, Supplier<Double> right) {
		m_left = left;
		m_right = right;
		m_climberSubsystem = subsystem;
		addRequirements(m_climberSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		leftSpeed = MathUtil.applyDeadband(m_left.get(), ControllerConstants.kDeadzone);
		rightSpeed = MathUtil.applyDeadband(m_right.get(), ControllerConstants.kDeadzone);

		m_climberSubsystem.setSpeed(leftSpeed, rightSpeed);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}
}
