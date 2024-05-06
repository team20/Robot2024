package frc.robot.commands.drive;

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class BangBangDriveDistanceCommand extends Command {
	private final DriveSubsystem m_driveSubsystem;
	private double m_target; // if distance, in meters; if angle, in degrees
	private double m_amount;
	private double m_tolerance;

	/***
	 * Autonomous command to drive straight
	 * 
	 * @param amount
	 *               amount is distance in meters
	 */
	public BangBangDriveDistanceCommand(DriveSubsystem subsystem, double amount, double tolerance) {
		m_driveSubsystem = subsystem;
		m_amount = amount;
		m_tolerance = tolerance;
		addRequirements(subsystem);
	}

	/***
	 * Autonomous command to drive straight
	 * 
	 * @param amount
	 *               amount is distance in meters
	 */
	public BangBangDriveDistanceCommand(DriveSubsystem subsystem, double amount) {
		m_driveSubsystem = subsystem;
		m_amount = amount;
		m_tolerance = 0.1;
		addRequirements(subsystem);
	}

	@Override
	public void initialize() {
		double currentPosition = m_driveSubsystem.getPose().getX();
		m_target = currentPosition + m_amount;
	}

	@Override
	public void execute() {
		double sign;
		if (m_target > m_driveSubsystem.getPose().getX()) {
			sign = 1;
		} else {
			sign = -1;
		}

		double error = getDiff();
		double kP = 1.2;
		double speed = error * kP;
		if (speed > kMaxSpeed) {
			speed = kMaxSpeed;
		} else if (speed < kMinSpeed) {
			speed = kMinSpeed;
		}

		m_driveSubsystem.setModuleStates(speed * sign, 0, 0, false);
	}

	@Override
	public boolean isFinished() {
		// Determine whether the target distance has been reached
		double diff = getDiff();
		// SmartDashboard.putNumber("diff", diff);
		return diff < m_tolerance;
	}

	@Override
	public void end(boolean interrupted) {
		m_driveSubsystem.stopDriving();
	}

	private double getDiff() {
		return Math.abs(m_target - m_driveSubsystem.getPose().getX());
	}
}