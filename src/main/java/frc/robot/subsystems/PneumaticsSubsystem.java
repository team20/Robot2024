// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.PneumaticsConstants.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticsSubsystem extends SubsystemBase {
	private final PneumaticHub m_hub = new PneumaticHub(kPneumaticHubID);
	private final DoubleSolenoid m_leftAmpBarSolenoid = m_hub.makeDoubleSolenoid(kLeftAmpBarForwardChannel,
			kLeftAmpBarReverseChannel);
	private final DoubleSolenoid m_rightAmpBarSolenoid = m_hub.makeDoubleSolenoid(kRightAmpBarForwardChannel,
			kRightAmpBarReverseChannel);
	private final DoubleSolenoid m_leftIntakeSolenoid = m_hub.makeDoubleSolenoid(kLeftIntakeForwardChannel,
			kLeftIntakeReverseChannel);
	private final DoubleSolenoid m_rightIntakeSolenoid = m_hub.makeDoubleSolenoid(kRightIntakeForwardChannel,
			kRightIntakeReverseChannel);
	private boolean m_ampBarExtended;
	private boolean m_intakeExtended;

	/** Creates a new PneumaticsSubsystem. */
	public PneumaticsSubsystem() {
	}

	/**
	 * Creates a command to toggle the intake.
	 * 
	 * @return The command.
	 */
	public Command toggleIntakeCommand() {
		return runOnce(() -> {
			if (!m_intakeExtended) {
				m_leftIntakeSolenoid.set(kIntakeUp);
				m_rightIntakeSolenoid.set(kIntakeUp);
				m_intakeExtended = true;
			} else {
				m_leftIntakeSolenoid.toggle();
				m_rightIntakeSolenoid.toggle();
			}
		});
	}

	/**
	 * Creates a command to raise the intake.
	 * 
	 * @return The command.
	 */
	public Command upIntakeCommand() {
		return runOnce(() -> {
			m_leftIntakeSolenoid.set(kIntakeUp);
			m_rightIntakeSolenoid.set(kIntakeUp);
		});
	}

	/**
	 * Creates a command to raise the intake.
	 * 
	 * @return The command.
	 */
	public Command downIntakeCommand() {
		return runOnce(() -> {
			m_leftIntakeSolenoid.set(kIntakeDown);
			m_rightIntakeSolenoid.set(kIntakeDown);
		});
	}

	/**
	 * Creates a command to toggle the amp bar.
	 * 
	 * @return The command.
	 */
	public Command toggleAmpBarCommand() {
		return runOnce(() -> {
			if (!m_ampBarExtended) {
				m_leftAmpBarSolenoid.set(Value.kForward);
				m_rightAmpBarSolenoid.set(Value.kForward);
				m_ampBarExtended = true;
			} else {
				m_leftAmpBarSolenoid.toggle();
				m_rightAmpBarSolenoid.toggle();
			}
		});
	}

	/**
	 * Creates a command to extend the amp bar.
	 * 
	 * @return The command.
	 */
	public Command extendAmpBarCommand() {
		return runOnce(() -> {
			m_leftAmpBarSolenoid.set(Value.kForward);
			m_rightAmpBarSolenoid.set(Value.kForward);

		});
	}

	/**
	 * Creates a command to retract the amp bar.
	 * 
	 * @return The command.
	 */
	public Command retractAmpBarCommand() {
		return runOnce(() -> {
			m_leftAmpBarSolenoid.set(Value.kReverse);
			m_rightAmpBarSolenoid.set(Value.kReverse);
		});
	}

	public void periodic() {
		SmartDashboard.putNumber("Pressure", m_hub.getPressure(0));
		// SmartDashboard.putNumber("Pressure (Other side)",m_hub.getPressure(1));
	}
}
