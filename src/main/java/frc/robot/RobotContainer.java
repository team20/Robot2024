// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ControllerConstants.Axis;
import frc.robot.commands.climber.ClimberDriveCommand;
import frc.robot.commands.climber.ClimberPresetCommand;
import frc.robot.commands.climber.ClimberPresetCommand.ClimberOperation;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	private final CommandGenericHID m_driverController = new CommandGenericHID(
			ControllerConstants.kDriverControllerPort);
	private final CommandGenericHID m_operatorController = new CommandGenericHID(
			ControllerConstants.kOperatorControllerPort);
	// private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private final ArduinoSubsystem m_arduinoSubsystem = new ArduinoSubsystem();
	private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
	private final SendableChooser<Command> m_autoSelector = new SendableChooser<Command>();
	// private final FlywheelSubsystem m_flywheelSubsystem = new
	// FlywheelSubsystem();

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */

	public RobotContainer() {
		// Configure the button bindings
		// m_autoSelector.addOption("Test Steering",
		// SetSteeringCommand.getCalibrationCommand(m_driveSubsystem));
		// m_autoSelector.addOption("PID Turn 90 degrees", new
		// PIDTurnCommand(m_driveSubsystem, 90, 0.5));
		// m_autoSelector.addOption("Bang Bang Drive 2 Meters",
		// new BangBangDriveDistanceCommand(m_driveSubsystem, 2, 0.01));
		// m_autoSelector.addOption("PID Drive 2 Meters",
		// DriveDistanceCommand.create(m_driveSubsystem, 3.0, 0.01));
		// m_autoSelector.addOption("Knock Over Blocks",
		// CommandComposer.getBlocksAuto(m_driveSubsystem));

		SmartDashboard.putData(m_autoSelector);
		configureButtonBindings();
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by instantiating a {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
	 * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		m_climberSubsystem.setDefaultCommand(new ClimberDriveCommand(m_climberSubsystem,
				() -> m_operatorController.getRawAxis(Axis.kLeftY),
				() -> m_operatorController.getRawAxis(Axis.kRightY)));

		m_operatorController.povUp()
				.onTrue(new ClimberPresetCommand(m_climberSubsystem, ClimberOperation.TOP));
		m_operatorController.povDown()
				.onTrue(new ClimberPresetCommand(m_climberSubsystem, ClimberOperation.ZERO));

		// m_controller.button(Button.kCircle).onTrue(m_driveSubsystem.resetHeadingCommand());
		// m_controller.button(Button.kTriangle).onTrue(m_driveSubsystem.alignModulesToZeroComamnd());
		// m_controller.button(Button.kSquare).onTrue(m_driveSubsystem.resetEncodersCommand());
		// m_controller.button(Button.kX).onTrue(new
		// DriveDistanceCommand(m_driveSubsystem, 10, 0.01));
		// Should have RainbowPartyFunTime in the last 20 seconds of a match
		// TODO: Check if this can be overridden LED buttons
		// new Trigger(() -> DriverStation.getMatchTime() <= 20)
		// .onTrue(m_arduinoSubsystem.writeStatus(StatusCode.RAINBOW_PARTY_FUN_TIME));
		// TODO: LEDs to add: Left Trigger -> Orange LED, with other stuff, BLUE WHEN
		// SHOOT COMMANDS ARE DONE

		// LEDs for when you want AMP
		// m_operatorController.povLeft().onTrue(m_arduinoSubsystem.writeStatus(StatusCode.BLINKING_PURPLE));
		// LEDs for when you want COOP
		// m_operatorController.povUp().onTrue(m_arduinoSubsystem.writeStatus(StatusCode.BLINKING_YELLOW));
		// LEDs for when you want HP to drop a note
		// m_operatorController.povRight().onTrue(m_arduinoSubsystem.writeStatus(StatusCode.BLINKING_RED));
		// DEFAULT Button
		// m_operatorController.povDown().onTrue(m_arduinoSubsystem.writeStatus(StatusCode.DEFAULT));
		// RainbowPartyFunTime
		// m_operatorController.button(Button.kShare)
		// .onTrue(m_arduinoSubsystem.writeStatus(StatusCode.RAINBOW_PARTY_FUN_TIME));

		// m_driveSubsystem.setDefaultCommand(m_driveSubsystem.driveCommand(
		// () -> m_driverController.getRawAxis(Axis.kLeftY),
		// () -> m_driverController.getRawAxis(Axis.kLeftX),
		// () -> m_driverController.getRawAxis(Axis.kRightTrigger),
		// () -> m_driverController.getRawAxis(Axis.kLeftTrigger)));
		// m_driverController.button(Button.kCircle).onTrue(m_driveSubsystem.resetHeadingCommand());
		// m_driverController.button(Button.kTriangle)
		// .onTrue(new FlywheelCommand(m_flywheelSubsystem,
		// FlywheelOperation.SET_VELOCITY,
		// 200)); // 200 w/ gearbox on valk puts this at about 2 rotation per second
		// m_driverController.button(Button.kSquare).onTrue(m_driveSubsystem.resetEncodersCommand());
		// m_driverController.button(Button.kX).onTrue(new
		// DriveDistanceCommand(m_driveSubsystem, 10, 0.01));
	}

	public Command getAutonomousCommand() {
		return m_autoSelector.getSelected();
	}
}