package frc.robot.commands.drive;

import static frc.robot.Constants.AimerConstants.*;
import static frc.robot.Constants.DriveConstants.*;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.AimerSubsystem;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.ArduinoSubsystem.StatusCode;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.LimeLightSubsystem.Pose;

/**
 * The {@code DriveWhileAimingCommand} is responsible for moving the robot such
 * that the
 * robot always faces toward a certain target.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public class DriveWhileAimingExtendedCommand extends Command {

	/**
	 * The supplier that provides the forward speed in meters per second.
	 */
	private Supplier<Double> m_forwardSpeed;

	/**
	 * The supplier that provides the strafe speed in meters per second.
	 */
	private Supplier<Double> m_strafeSpeed;

	/**
	 * The {@code ProfiledPIDController} for controlling the robot in the yaw
	 * dimension in angles.
	 */
	private ProfiledPIDController m_controllerYaw;

	/**
	 * The {@code DriveSubsystem} used by this {@code DriveWhileAimingCommand}.
	 */
	private DriveSubsystem m_driveSubsystem;

	/**
	 * The {@code AimerSubsystem} used by this {@code DriveWhileAimingCommand}.
	 */
	private AimerSubsystem m_aimerSubsystem;

	/**
	 * The {@code FlywheelSubsystem} used by this {@code DriveWhileAimingCommand}.
	 */
	private FlywheelSubsystem m_flywheelSubsystem;

	/**
	 * The {@code ArduinoSubsystem} used by this {@code DriveWhileAimingCommand}.
	 */
	private ArduinoSubsystem m_arduinoSubsystem;

	/**
	 * The {@code LimeLightSubsystem} used by this {@code DriveWhileAimingCommand}.
	 */
	private LimeLightSubsystem m_limeLightSubsystem;

	/**
	 * The timestamp value used for periodic reporting.
	 */
	private Long timestamp = null;

	/**
	 * The shooting delay.
	 */
	private double m_shootingDelay;

	/**
	 * The impact of distance changes on aiming (< 1).
	 */
	private double m_distanceSensitivity;

	/**
	 * The position of the robot estimated previously.
	 */
	private Translation2d m_previousPosition = null;

	/**
	 * The estimated velocity of the robot.
	 */
	private Translation2d m_velocity = null;

	/**
	 * The {@code Supplier} that calculates the aiming target.
	 */
	private Supplier<Translation2d> m_targetSupplier;

	/**
	 * Constructs a new {@code DriveWhileAimingCommand} whose purpose is to move the
	 * robot to a certain target pose.
	 * 
	 * @param forwardSpeed        the supplier that provides the forward speed in
	 *                            meters
	 *                            per second
	 * @param strafeSpeed         the supplier that provides the strafe speed in
	 *                            meters per second
	 * @param angleTolerance      the angle error in degrees which is tolerable
	 * @param targetSupplier      the {@code Supplier} that calculates the aiming
	 *                            target
	 * @param shootingDelay       the shooting delay in seconds
	 * @param distanceSensitivity the impact of distance changes on aiming (< 1)
	 * @param driveSubsystem      the {@code DriveSubsystem} used by the
	 *                            {@code DriveWhileAimingCommand}
	 * @param aimerSubsystem      the {@code AimerSubsystem} used by the
	 *                            {@code DriveWhileAimingCommand}
	 * @param flywheelSubsystem   the {@code FlywheelSubsystem} used by the
	 *                            {@code DriveWhileAimingCommand}
	 * @param arduinoSubsystem    the {@code ArduinoSubsystem} used by the
	 *                            {@code DriveWhileAimingCommand}
	 * @param limeLightSubsystem  the {@code LimeLightSubsystem} used by the
	 *                            {@code DriveWhileAimingCommand}
	 */
	public DriveWhileAimingExtendedCommand(Supplier<Double> forwardSpeed, Supplier<Double> strafeSpeed,
			Supplier<Translation2d> targetSupplier,
			double angleTolerance, double shootingDelay, double distanceSensitivity, DriveSubsystem driveSubsystem,
			AimerSubsystem aimerSubsystem,
			FlywheelSubsystem flywheelSubsystem, ArduinoSubsystem arduinoSubsystem,
			LimeLightSubsystem limeLightSubsystem) {
		m_forwardSpeed = forwardSpeed;
		m_strafeSpeed = strafeSpeed;
		m_targetSupplier = targetSupplier;
		m_shootingDelay = shootingDelay;
		m_distanceSensitivity = distanceSensitivity;
		m_controllerYaw = new ProfiledPIDController(kTurnP * 1.5, kTurnI, kTurnD,
				new TrapezoidProfile.Constraints(kTurnMaxVelocity, kTurnMaxAcceleration * 1.5));
		m_controllerYaw.setTolerance(angleTolerance);
		m_controllerYaw.enableContinuousInput(-180, 180);
		m_controllerYaw.setGoal(0);
		m_driveSubsystem = driveSubsystem;
		m_aimerSubsystem = aimerSubsystem;
		m_flywheelSubsystem = flywheelSubsystem;
		m_arduinoSubsystem = arduinoSubsystem;
		m_limeLightSubsystem = limeLightSubsystem;
		addRequirements(m_driveSubsystem, aimerSubsystem, arduinoSubsystem);
		if (flywheelSubsystem != null)
			addRequirements(flywheelSubsystem);
	}

	/**
	 * Is invoked at the commencement of this {@code DriveWhileAimingCommand} (i.e,
	 * when the scheduler begins to periodically execute this
	 * {@code DriveWhileAimingCommand}).
	 */

	@Override
	public void initialize() {
		m_controllerYaw.reset(0);
		if (m_flywheelSubsystem != null) {
			m_flywheelSubsystem.setTopVelocity(4850);
			m_flywheelSubsystem.setBottomVelocity(4800);
		}
		m_previousPosition = null;
		m_velocity = null;
		m_aimerSubsystem.setAimerHeight(kPassActuatorHeight);
	}

	/**
	 * Is invoked periodically by the scheduler until this
	 * {@code DriveWhileAimingCommand} is either ended or interrupted.
	 */
	@Override
	public void execute() {
		double fwdSpeed = kTeleopMaxVoltage
				* MathUtil.applyDeadband(m_forwardSpeed.get(), ControllerConstants.kDeadzone);
		fwdSpeed = Math.signum(fwdSpeed) * (fwdSpeed * fwdSpeed);
		double strSpeed = kTeleopMaxVoltage
				* MathUtil.applyDeadband(m_strafeSpeed.get(), ControllerConstants.kDeadzone);
		strSpeed = Math.signum(strSpeed) * (strSpeed * strSpeed);
		double rotSpeed = 0;
		try {
			var position = m_limeLightSubsystem.estimatedPose().getTranslation();
			if (m_previousPosition != null) {
				Translation2d velocity = position
						.minus(m_previousPosition);
				m_velocity = m_velocity == null ? velocity
						: new Translation2d(velocity.getX() * 0.2 + m_velocity.getX() * 0.8,
								velocity.getY() * 0.2 + m_velocity.getY() * 0.8);
			}
			m_previousPosition = position;
			var transform = transformationToTarget();
			double angle = transform.getRotation().getDegrees();
			rotSpeed = m_controllerYaw.calculate(-angle); // to achieve angle error 0
			if (timestamp == null || timestamp < System.currentTimeMillis()) {
				double readiness = m_limeLightSubsystem.confidence()
						+ (m_aimerSubsystem.atAimerSetpoint() ? 0 : -1);
				SmartDashboard.putNumber(
						"pose estimation: drive while aiming ", readiness);
				if (readiness > 0.5) {
					timestamp = System.currentTimeMillis() + 500;
					m_arduinoSubsystem.setCode(StatusCode.SOLID_BLUE);
				}
			}
		} catch (Exception e) {
			e.printStackTrace();
		}
		// NEGATION if positive turnSpeed: clockwise rotation
		rotSpeed = -rotSpeed;
		m_driveSubsystem.setModuleStates(fwdSpeed, strSpeed, rotSpeed, true);
	}

	/**
	 * Returns the transformation for shooting to the target.
	 * 
	 * @return the transformation for shooting to the target
	 */
	private Transform2d transformationToTarget() {
		var target = m_targetSupplier.get();
		var pose = m_limeLightSubsystem.estimatedPose();
		if (m_velocity != null) {
			var velocity = m_velocity.times(m_shootingDelay / TimedRobot.kDefaultPeriod);
			pose = new Pose(velocity.getX(), velocity.getY(), 0).add(pose);
			var distance = LimeLightSubsystem.transformationToward(target, 0,
					pose).getTranslation().getNorm();
			velocity = m_velocity.times(m_distanceSensitivity * distance / 10 /
					TimedRobot.kDefaultPeriod);
			target = new Translation2d(target.getX() - velocity.getX(), target.getY() - velocity.getY());
		}
		return LimeLightSubsystem.transformationToward(target, 0, pose);
	}

	/**
	 * Is invoked once this {@code DriveWhileAimingCommand} is either ended or
	 * interrupted.
	 * 
	 * @param interrupted indicates if this {@code DriveWhileAimingCommand} was
	 *                    interrupted
	 */
	@Override
	public void end(boolean interrupted) {
		if (m_flywheelSubsystem != null)
			m_flywheelSubsystem.stopFlywheel();
	}

}