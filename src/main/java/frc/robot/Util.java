package frc.robot;

import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;

public class Util {
	/**
	 * Optimizes CAN bus usage by effectively eliminating Spark Max certain status
	 * frames from being sent (this will give them a period of ~30 seconds).
	 * 
	 * <p>
	 * Note that if you are using an absolute encoder, an alternate encoder, or an
	 * analog encoder (for some reason), you will need to set the frame periods for
	 * those device's data manually.
	 * 
	 * @param sparkMax The Spark Max to optimize.
	 */
	public static void optimizeCanUsage(CANSparkMax sparkMax) {
		sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 28500);
		sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 29500);
		sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 30500);
		sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 31500);
		sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 32500);
	}

	/**
	 * Increases the rate at which Spark Max data is sent. Currently increases
	 * update rate for encoder position to 100 Hz.
	 * 
	 * @param sparkMax The Spark Max.
	 */
	public static void increaseMotorDataFrequency(CANSparkMax sparkMax) {
		sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);
	}
}
