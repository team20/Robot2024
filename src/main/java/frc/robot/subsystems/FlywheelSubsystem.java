package frc.robot.subsystems;

import static frc.robot.Constants.FlywheelConstants.*;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlywheelSubsystem extends SubsystemBase {
	private final CANSparkMax m_neoFlywheelBottom = new CANSparkMax(kBottomPort, MotorType.kBrushless);
	private final CANSparkMax m_neoFlywheelTop = new CANSparkMax(kTopPort, MotorType.kBrushless);
	private final SparkPIDController m_neoControllerBottom = m_neoFlywheelBottom.getPIDController();
	private final SparkPIDController m_neoControllerTop = m_neoFlywheelTop.getPIDController();

	private final RelativeEncoder m_neoEncoderBottom = m_neoFlywheelBottom.getEncoder();
	private final RelativeEncoder m_neoEncoderTop = m_neoFlywheelTop.getEncoder();

	private double m_setVelocity;

	/**
	 * Initializes a new instance of the {@link FlywheelSubsystem} class.
	 */
	public FlywheelSubsystem() {
		// Initialize Motors
		m_neoFlywheelBottom.restoreFactoryDefaults();
		m_neoFlywheelBottom.setInverted(kBottomInvert);
		m_neoFlywheelBottom.setIdleMode(IdleMode.kCoast);
		m_neoFlywheelBottom.enableVoltageCompensation(12);
		m_neoFlywheelBottom.setSmartCurrentLimit(kSmartCurrentLimit);
		m_neoFlywheelBottom.setSecondaryCurrentLimit(kPeakCurrentLimit,
				kPeakCurrentDurationMillis);

		m_neoFlywheelTop.restoreFactoryDefaults();
		m_neoFlywheelTop.setInverted(kTopInvert);
		m_neoFlywheelTop.setIdleMode(IdleMode.kCoast);
		m_neoFlywheelTop.enableVoltageCompensation(12);
		m_neoFlywheelTop.setSmartCurrentLimit(kSmartCurrentLimit);
		m_neoFlywheelTop.setSecondaryCurrentLimit(kPeakCurrentLimit,
				kPeakCurrentDurationMillis);
		// m_neoFlywheeltop.follow(m_neoFlywheelBottom, ktopOppose);

		m_neoEncoderBottom.setVelocityConversionFactor(kGearRatio);
		m_neoControllerBottom.setP(kP);
		m_neoControllerBottom.setI(kI);
		m_neoControllerBottom.setD(kD);
		m_neoControllerBottom.setIZone(kIz);
		m_neoControllerBottom.setFF(kFF);
		m_neoControllerBottom.setOutputRange(kMinOutput, kMaxOutput);

		m_neoEncoderTop.setVelocityConversionFactor(kGearRatio);
		m_neoControllerTop.setP(kP);
		m_neoControllerTop.setI(kI);
		m_neoControllerTop.setD(kD);
		m_neoControllerTop.setIZone(kIz);
		m_neoControllerTop.setFF(kFF);
		m_neoControllerTop.setOutputRange(kMinOutput, kMaxOutput);
	}

	public void periodic() {
		SmartDashboard.putNumber("Flywheel Bottom Velocity", getBottomVelocity());
		SmartDashboard.putNumber("Flywheel top Velocity", getTopVelocity());
		SmartDashboard.putNumber("Flywheel Bottom Current", m_neoFlywheelBottom.getOutputCurrent());
		SmartDashboard.putNumber("Flywheel top Current", m_neoFlywheelTop.getOutputCurrent());
		// essentially the end method of the flywheel velocity setpoint mode
		if (m_setVelocity == 0 && Math.abs(m_neoEncoderBottom.getVelocity()) > 0.05) {
			m_neoFlywheelBottom.stopMotor();
			m_neoFlywheelTop.stopMotor();
		}
	}

	/**
	 * @return Current setpoint.
	 */
	public double getSetpoint() {
		return m_setVelocity;
	}

	/**
	 * @return Measured velocity.
	 */
	public double getBottomVelocity() {
		return m_neoEncoderBottom.getVelocity();
	}

	public double getTopVelocity() {
		return m_neoEncoderTop.getVelocity();
	}

	/**
	 * Sets target speed for flywheel.
	 * 
	 * @param velocity Target velocity (rpm).
	 */
	public void setBottomVelocity(double velocity) {
		m_setVelocity = velocity;
		m_neoControllerBottom.setReference(m_setVelocity, ControlType.kVelocity);
	}

	public void setTopVelocity(double velocity) {
		m_setVelocity = velocity;
		m_neoControllerTop.setReference(m_setVelocity, ControlType.kVelocity);
	}

	/**
	 * @return Whether the flywheel is at its setpoint ABOVE 0
	 */
	public Command stopFlywheel() {
		return runOnce(() -> {
			m_neoFlywheelBottom.set(0);
			m_neoFlywheelTop.set(0);
		});
	}

	/**
	 * @return Whether the flywheel is at its setpoint ABOVE 0
	 */
	public boolean atSetpoint() {
		return Math.abs(getBottomVelocity() - getSetpoint()) < kAllowedError
				|| Math.abs(getTopVelocity() - getSetpoint()) < kAllowedError;
	}

}