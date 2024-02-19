package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
	private final CANSparkMax m_neoShooter = new CANSparkMax(FlywheelConstants.kFlywheelMasterPort,
			MotorType.kBrushless); // TODO make 550s
	private final SparkPIDController m_neoController = m_neoShooter.getPIDController();
	private final RelativeEncoder m_neoEncoderMaster = m_neoShooter.getEncoder();
	private double m_setVelocity;
	private double m_kShooterTolerance = Constants.ShooterConstants.kShooterTolerance;
	private double m_actuatorHeightSetpoint;
	private double m_conversionFactor;

	/**
	 * Initializes a new instance of the {@link FlywheelSubsystem} class.
	 */
	public ShooterSubsystem() {
		// Initialize Motors
		m_neoShooter.restoreFactoryDefaults();
		m_neoShooter.setInverted(ShooterConstants.kMasterInvert);
		m_neoShooter.setIdleMode(IdleMode.kCoast);
		m_neoShooter.enableVoltageCompensation(12);
		m_neoShooter.setSmartCurrentLimit(ShooterConstants.kSmartCurrentLimit);
		m_neoShooter.setSecondaryCurrentLimit(ShooterConstants.kPeakCurrentLimit,
				ShooterConstants.kPeakCurrentDurationMillis);

		m_conversionFactor = 1 / ShooterConstants.kGearRatio;
		float leadScrewSoftLimit = (float) (ShooterConstants.kLeadScrewLimit * m_conversionFactor);
		m_neoShooter.setSoftLimit(SoftLimitDirection.kForward, leadScrewSoftLimit);

		m_neoController.setP(ShooterConstants.kP);
		m_neoController.setI(ShooterConstants.kI);
		m_neoController.setD(ShooterConstants.kD);
		m_neoController.setIZone(ShooterConstants.kIz);
		m_neoController.setFF(ShooterConstants.kFF);
		m_neoController.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);
	}

	public void periodic() {

	}

	/**
	 * @return Measured velocity.
	 */
	public double getVelocity() {
		return m_neoEncoderMaster.getVelocity();
	}

	/**
	 * Sets target speed for shooter.
	 * 
	 * @param velocity Target velocity (rpm).
	 */
	public void setVelocity(double velocity) {
		m_setVelocity = velocity;
		m_neoController.setReference(m_setVelocity, ControlType.kVelocity);
	}

	public void stopMotors() {
		setVelocity(0);
	}

	public double getActuatorHeight() {
		return m_neoEncoderMaster.getPosition(); // m_neoEncoderMaster.getPosition();

	}

	public void setActuatorHeight(double actuatorHeightSetpoint) {
		m_actuatorHeightSetpoint = actuatorHeightSetpoint;
		m_neoController.setReference(actuatorHeightSetpoint, CANSparkMax.ControlType.kPosition, 0);
	}

	public void adjustActuatorSetpoint(double adjustAmount) {
		m_actuatorHeightSetpoint += adjustAmount;
		setActuatorHeight(m_actuatorHeightSetpoint);
	}

	public void setSpeed(double speed) {
		m_neoShooter.set(speed);
	}

	public boolean atActuatorSetpoint() {
		return (Math.abs(m_actuatorHeightSetpoint - getActuatorHeight()) <= m_kShooterTolerance);
	}
}