// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArduinoSubsystem extends SubsystemBase {
	/** The USB port that's used to connect to the Arduino. */
	private SerialPort m_usb;

	/** The bytes that control the LED mode */
	public enum StatusCode {
		RESET((byte) 0),
		RAINBOW_PARTY_FUN_TIME((byte) 1),
		SOLID_ORANGE((byte) 2),
		SOLID_BLUE((byte) 3),
		BLINKING_YELLOW((byte) 5),
		BLINKING_PURPLE((byte) 6),
		BLINKING_RED((byte) 7),
		DEFAULT((byte) 20);

		public byte code;

		private StatusCode(byte c) {
			code = c;
		}
	}

	/** Creates a new ArduinoSubsystem. */
	public ArduinoSubsystem() {
		try {
			m_usb = new SerialPort(9600, Port.kUSB);
		} catch (Exception e) {
			DriverStation.reportError("Could not initialize Arduino over USB", false);
			m_usb = null;
		}
		setCode(StatusCode.DEFAULT);
	}

	public void setCode(StatusCode code) {
		if (m_usb != null) {
			m_usb.write(new byte[] { code.code }, 1);
		}
	}

	public Command writeStatus(StatusCode code) {
		return runOnce(() -> setCode(code));
	}
}