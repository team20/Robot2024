package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ClampedController;
import frc.robot.subsystems.DriveSubsystem;

public class PolarDriveCommand extends Command {
	private final DriveSubsystem m_driveSubsystem;
	private double m_target;
	private double m_distance;
	private double m_tolerance;
	private double m_angle;
	private final ClampedController m_controller = new ClampedController(0.3, 0.2, .3);

	/**
	 * Creates a command to drive to a point by specifying a distance and and angle
	 * (robot relative).
	 * 
	 * @param subsystem The drive subsystem.
	 * @param distance  The distance to drive.
	 * @param angle     The angle to drive relative to the robot (CCW+).
	 * @param tolerance The distance tolerance.
	 */
	public PolarDriveCommand(DriveSubsystem subsystem, double distance, double angle, double tolerance) {
		m_driveSubsystem = subsystem;
		m_distance = distance;
		m_tolerance = tolerance;
		m_angle = angle;
		addRequirements(subsystem);
	}

	/**
	 * Creates a command to drive to a point by specifying a distance and and angle
	 * (robot relative).
	 * 
	 * @param subsystem The drive subsystem.
	 * @param distance  The distance to drive.
	 * @param angle     The angle to drive relative to the robot (CCW+).
	 */
	public PolarDriveCommand(DriveSubsystem subsystem, double amount, double angle) {
		this(subsystem, amount, angle, 0.01);
	}

	@Override
	public void initialize() {
		m_target = m_driveSubsystem.getModulePositions()[0].distanceMeters + m_distance;
		m_controller.setSetpoint(m_target);
		m_controller.setTolerance(m_tolerance);
	}

	@Override
	public void execute() {
		double speed = m_controller.calculate(m_driveSubsystem.getModulePositions()[0].distanceMeters);
		m_driveSubsystem.setModuleStatesDirect(new SwerveModuleState(speed, Rotation2d.fromDegrees(m_angle)));
	}

	@Override
	public boolean isFinished() {
		// Determine whether the target distance has been reached
		return m_controller.atSetpoint();
	}

	@Override
	public void end(boolean interrupted) {
		m_driveSubsystem.stopDriving();
	}
}