// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class SetSteeringCommand extends Command {
	private DriveSubsystem m_subsystem;
	private double m_angle;

	/** Creates a new SetSteering. */
	public SetSteeringCommand(DriveSubsystem subsystem, double angle) {
		m_subsystem = subsystem;
		m_angle = angle;
		addRequirements(subsystem);
	}

	public static Command getCalibrationCommand(DriveSubsystem subsystem) {
		return new SequentialCommandGroup(
				new SetSteeringCommand(subsystem, 45),
				new SetSteeringCommand(subsystem, 0),
				new SetSteeringCommand(subsystem, 45));
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_subsystem.setModuleAngles(m_angle);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		boolean FLWithinTolerance = MathUtil.isNear(m_angle, m_subsystem.getModulePositions()[0].angle.getDegrees(), 1);
		boolean FRWithinTolerance = MathUtil.isNear(m_angle, m_subsystem.getModulePositions()[1].angle.getDegrees(), 1);
		boolean BLWithinTolerance = MathUtil.isNear(m_angle, m_subsystem.getModulePositions()[2].angle.getDegrees(), 1);
		boolean BRWithinTolerance = MathUtil.isNear(m_angle, m_subsystem.getModulePositions()[3].angle.getDegrees(), 1);
		SmartDashboard.putNumber("FL Module Angle", m_subsystem.getModulePositions()[0].angle.getDegrees());
		SmartDashboard.putNumber("FR Module Angle", m_subsystem.getModulePositions()[1].angle.getDegrees());
		SmartDashboard.putNumber("BL Module Angle", m_subsystem.getModulePositions()[2].angle.getDegrees());
		SmartDashboard.putNumber("BR Module Angle", m_subsystem.getModulePositions()[3].angle.getDegrees());
		return FLWithinTolerance && FRWithinTolerance && BLWithinTolerance && BRWithinTolerance;
	}
}
