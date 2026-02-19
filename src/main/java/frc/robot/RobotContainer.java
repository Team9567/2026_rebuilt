// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.LEDSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot} periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  SendableChooser<Command> autochooser = new SendableChooser<>();
  private final FuelSubsystem m_fuelSubsystem = new FuelSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();

  private final LEDSubsystem m_ledSubsystem = new LEDSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandGenericHID m_driverController = new CommandGenericHID(OperatorConstants.kDriverControllerPort);
  private final CommandGenericHID m_controllerController = new CommandGenericHID(
      OperatorConstants.kControllerControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    autochooser.addOption("nothing", Commands.none());
    autochooser.addOption("exampleAuto", Autos.exampleAuto(m_drivetrainSubsystem));
    SmartDashboard.putData("AutoChooser", autochooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Set alliance color when robot is enabled
    new Trigger(DriverStation::isEnabled)
        .onTrue(new InstantCommand(m_ledSubsystem::setAllianceColor, m_ledSubsystem));

    // Dim LEDs when disabled
    new Trigger(DriverStation::isDisabled)
        .onTrue(new InstantCommand(m_ledSubsystem::setDisabled, m_ledSubsystem));

    m_drivetrainSubsystem.setDefaultCommand(new RunCommand(() -> {
      m_drivetrainSubsystem.arcadeDrive(-m_driverController.getRawAxis(OperatorConstants.kControllerLeftVertical),
          -m_driverController.getRawAxis(OperatorConstants.kControllerRightHorizontal));
    }, m_drivetrainSubsystem));

    Trigger lowGear = m_driverController.button(OperatorConstants.kDriverControllerY);
    lowGear.onTrue(new InstantCommand(m_ledSubsystem::setLowGear, m_ledSubsystem))
        .onFalse(new InstantCommand(m_ledSubsystem::setHighGear, m_ledSubsystem));
    m_drivetrainSubsystem.setGearTrigger(lowGear);

    m_controllerController.axisGreaterThan(OperatorConstants.kDriverControllerRightTrigger, 0.90).whileTrue(m_fuelSubsystem.shootVelocityCommand(60));
    m_controllerController.axisGreaterThan(OperatorConstants.kDriverControllerLeftTrigger, 0.80).whileTrue(m_fuelSubsystem.spinupCommand());
    m_controllerController.button(OperatorConstants.kDriverControllerLeftBumper).whileTrue(m_fuelSubsystem.intakeCommand());
    m_controllerController.button(OperatorConstants.kDriverControllerRightBumper).whileTrue(m_fuelSubsystem.ejectCommand());
    m_controllerController.button(OperatorConstants.kDriverControllerX).whileTrue(m_fuelSubsystem.shootDashboardVelocityCommand());
    m_controllerController.button(OperatorConstants.kDriverControllerStart).onTrue(m_climberSubsystem.homeCommand());
    m_controllerController.button(OperatorConstants.kDriverControllerY).onTrue(m_climberSubsystem.startClimbCommand());
    m_controllerController.button(OperatorConstants.kDriverControllerB).onTrue(m_climberSubsystem.hangCommand());
    m_controllerController.button(OperatorConstants.kDriverControllerA).onTrue(m_climberSubsystem.climbCommand());

    DoubleSupplier getGyroZValue = ()->{
      return m_drivetrainSubsystem.getGyroZValue();
    };

    m_climberSubsystem.setZSupplier(getGyroZValue); 
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_drivetrainSubsystem);
  }
}
