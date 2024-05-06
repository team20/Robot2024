package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ClampedController;
import frc.robot.subsystems.DriveSubsystem;

public class NinjaStarCommand extends Command {
	private final DriveSubsystem m_driveSubsystem;
	private Pose2d m_startPose;
	private double m_xAmount;
	private double m_yAmount;
	private double m_xTarget;
	private double m_yTarget;
	private double m_targetAngle;
	private final ClampedController m_xController = new ClampedController(3.6, 1.2, 3.6);
	private final ClampedController m_yController = new ClampedController(3.6, 1.2, 3.6);
	private final ClampedController m_rotController = new ClampedController(0.12, 1.2, 3.6);

	/**
	 * Creates a command to drive to a point by specifying a distance and and angle
	 * (robot relative).
	 * 
	 * @param subsystem The drive subsystem.
	 * @param distance  The distance to drive.
	 * @param angle     The angle to drive relative to the robot (CCW+).
	 * @param tolerance The distance tolerance.
	 */
	public NinjaStarCommand(DriveSubsystem subsystem, double xAmount, double yAmount, double targetAngle,
			double xTolerance, double yTolerance, double rotTolerance) {
		m_driveSubsystem = subsystem;
		m_xAmount = xAmount;
		m_yAmount = -yAmount; // negate so that right is positive (add to documentation)
		m_targetAngle = targetAngle;
		m_xController.setTolerance(xTolerance);
		m_yController.setTolerance(yTolerance);
		m_rotController.setTolerance(rotTolerance);
		m_rotController.enableContinuousInput(0, 360);
		// SmartDashboard.putString("ninja star command", "constructed");
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
	public NinjaStarCommand(DriveSubsystem subsystem, double xAmount, double yAmount, double targetAngle) {
		this(subsystem, xAmount, yAmount, targetAngle, 0.01, 0.01, 2);
	}

	@Override
	public void initialize() {
		// SmartDashboard.putString("ninja star command", "initialized");
		m_startPose = m_driveSubsystem.getPose();
		m_xTarget = m_startPose.getX() + m_xAmount;
		m_yTarget = m_startPose.getY() + m_yAmount;

		m_xController.setSetpoint(-m_xTarget);
		m_yController.setSetpoint(-m_yTarget);
		m_rotController.setSetpoint(m_targetAngle);
		// SmartDashboard.putNumber("target angle", m_targetAngle);
	}

	@Override
	public void execute() {
		double xSpeed = m_xController.calculate(m_driveSubsystem.getPose().getX());
		if (m_xController.atSetpoint()) {
			xSpeed = 0;
		}
		double ySpeed = m_yController.calculate(m_driveSubsystem.getPose().getY());
		if (m_yController.atSetpoint()) {
			ySpeed = 0;
		}
		double rotSpeed = -m_rotController.calculate(m_driveSubsystem.getHeading().getDegrees());
		if (m_rotController.atSetpoint()) {
			rotSpeed = 0;
		}
		// SmartDashboard.putNumber("rot error", m_rotController.getPositionError());
		m_driveSubsystem.setModuleStates(xSpeed, ySpeed, rotSpeed, true);
	}

	@Override
	public boolean isFinished() {
		// Determine whether the target pose and angle has been reached
		return (m_xController.atSetpoint() && m_yController.atSetpoint() && m_rotController.atSetpoint());
	}

	@Override
	public void end(boolean interrupted) {
		m_driveSubsystem.stopDriving();
	}
}