package frc.robot.subsystems;

import static frc.robot.Constants.AimerConstants.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ClampedController;
import frc.robot.Constants.AimerConstants;

public class AimerSubsystem extends SubsystemBase {
	private final CANSparkMax m_neoAimer = new CANSparkMax(AimerConstants.kAimerLeadScrewPort,
			MotorType.kBrushless);
	private final ClampedController m_controller = new ClampedController(kP, kMinAimerPower, kMaxAimerPower);
	private final CANcoder m_aimCancoder = new CANcoder(AimerConstants.kAimerEncoderPort);
	private double m_aimerHeightSetpoint;
	private boolean m_isManual;

	/**
	 * Initializes a new instance of the {@link AimerSubsystem} class.
	 */
	public AimerSubsystem() {
		// Initialize Motors
		m_neoAimer.restoreFactoryDefaults();
		m_neoAimer.setInverted(AimerConstants.kMasterInvert);
		m_neoAimer.setIdleMode(IdleMode.kBrake);
		m_neoAimer.enableVoltageCompensation(12);
		m_neoAimer.setSmartCurrentLimit(AimerConstants.kSmartCurrentLimit);
		m_neoAimer.setSecondaryCurrentLimit(AimerConstants.kPeakCurrentLimit,
				AimerConstants.kPeakCurrentDurationMillis);
		SparkLimitSwitch forwardSwitch = m_neoAimer.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
		forwardSwitch.enableLimitSwitch(true);
		SparkLimitSwitch reverseSwitch = m_neoAimer.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
		reverseSwitch.enableLimitSwitch(true);
		m_controller.setTolerance(AimerConstants.kAimerTolerance);
		this.setAimerHeight(kDefaultActuatorHeight);
	}

	public void periodic() {
		// If using a setpoint
		SmartDashboard.putNumber("Aimcoder Value", m_aimCancoder.getAbsolutePosition().getValueAsDouble());
		SmartDashboard.putNumber("Aimer Height", getAimerHeight());
		SmartDashboard.putNumber("Aimer 550 Amp Draw", m_neoAimer.getOutputCurrent());
		SmartDashboard.putNumber("Aimer Neo Speed", m_neoAimer.get());
		SmartDashboard.putNumber("Aimer 550 Temperature", m_neoAimer.getMotorTemperature());
		if (!m_isManual) {
			if (m_controller.atSetpoint()) {
				m_neoAimer.set(0);
			} else {
				m_neoAimer.set(m_controller.calculate(getAimerHeight()));
			}
		}
	}

	public void stopMotor() {
		setSpeed(0);
	}

	/**
	 * Gets the "height" of the aimer.
	 * 
	 * @return the "height" from 0-1 (cancoder value / maximum)
	 */
	public double getAimerHeight() {
		return m_aimCancoder.getAbsolutePosition().getValueAsDouble() / AimerConstants.kAimerMaxEncoderValue;
	}

	/**
	 * Sets the "height" of the aimer
	 * 
	 * @param the angle of the shooter from 0-1 (min to max)
	 */
	public void setAimerHeight(double actuatorHeightSetpoint) {
		m_aimerHeightSetpoint = actuatorHeightSetpoint;
		m_controller.setSetpoint(m_aimerHeightSetpoint);
	}

	// public void adjustAimerSetpoint(double adjustAmount) {
	// m_aimerHeightSetpoint += adjustAmount;
	// setAimerHeight(m_aimerHeightSetpoint);
	// }

	public void setSpeed(double speed) {
		speed = Math.signum(speed) * MathUtil.clamp(Math.abs(speed), 0, AimerConstants.kMaxAimerPower);
		m_neoAimer.set(speed);
	}

	public boolean atAimerSetpoint() {
		return m_controller.atSetpoint();
	}

	public void setManual(boolean isManual) {
		m_isManual = isManual;
	}
}