package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.FuelConstants;

public class FuelSubsystem extends SubsystemBase {
  private SparkMax fuelShooterMotor;
  private SparkMax fuelIntakeMotor;

  public FuelSubsystem() {
    if (FuelConstants.k_isEnabled) {
      // Set up the shooter motor as a brushless motor
      fuelShooterMotor = new SparkMax(FuelConstants.kShooterMotorID, MotorType.kBrushless);

      // Set up the Intake motor as a brushless motor
      fuelIntakeMotor = new SparkMax(FuelConstants.kIntakeMotorID, MotorType.kBrushless);

      // Create and apply configuration for shooter motor. Voltage compensation helps
      // the shooter behave the same as the battery
      // voltage dips. The current limit helps prevent breaker trips or burning out
      // the motor in the event the shooter stalls.
      SparkMaxConfig shooterConfig = new SparkMaxConfig();
      shooterConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(FuelConstants.kShooterMotorCurrentLimit)
          .voltageCompensation(FuelConstants.kShooterMotorVoltageCompens)
          .inverted(true);
      shooterConfig.softLimit
          .forwardSoftLimitEnabled(false)
          .reverseSoftLimitEnabled(false);
      shooterConfig.encoder.positionConversionFactor(FuelConstants.kShooterGearRatio);
      shooterConfig.encoder.velocityConversionFactor(1.0 / 60.0);
      fuelShooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      fuelShooterMotor.setCANTimeout(250);

      // Create and apply configuration for intake motor. Voltage compensation helps
      // the intake behave the same as the battery
      // voltage dips. The current limit helps prevent breaker trips or burning out
      // the motor in the event the intake stalls.
      SparkMaxConfig intakeConfig = new SparkMaxConfig();
      intakeConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(FuelConstants.kIntakeMotorCurrentLimit)
          .voltageCompensation(FuelConstants.kIntakeMotorVoltageCompens)
          .inverted(true);
      intakeConfig.softLimit
          .forwardSoftLimitEnabled(false)
          .reverseSoftLimitEnabled(false);
      intakeConfig.encoder.positionConversionFactor(FuelConstants.kIntakeGearRatio);
      intakeConfig.encoder.velocityConversionFactor(1.0 / 60.0);
      fuelIntakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      fuelIntakeMotor.setCANTimeout(250);
    }
  }

  public void set(double shooterSpeed, double intakeSpeed) {
    fuelShooterMotor.set(shooterSpeed);

    fuelIntakeMotor.set(intakeSpeed);

    SmartDashboard.putNumber("fuel/intakespeed", shooterSpeed);
    SmartDashboard.putNumber("fuel/shooterspeed", intakeSpeed);
  }

  public void stop() {
    fuelShooterMotor.set(0);
    fuelIntakeMotor.set(0);
  }

  public void setVoltage(double shooterVoltage, double intakeVoltage) {
    fuelShooterMotor.setVoltage(shooterVoltage);

    fuelIntakeMotor.setVoltage(intakeVoltage);

    SmartDashboard.putNumber("fuel/intakevoltage", shooterVoltage);
    SmartDashboard.putNumber("fuel/shootervoltage", intakeVoltage);
  }

  public Command setCommand(double shooterSpeed, double intakeSpeed) {
    if (FuelConstants.k_isEnabled) {
      return Commands.run(
          () -> {
            set(shooterSpeed, intakeSpeed);
          }, this);
    } else {
      return Commands.none();
    }
  }

  public Command setVoltageCommand(double shooterVoltage, double intakeVoltage) {
    if (FuelConstants.k_isEnabled) {
      return Commands.run(
          () -> {
            set(shooterVoltage, intakeVoltage);
          }, this);
    } else {
      return Commands.none();
    }
  }

  public Command stopCommand() {
    if (FuelConstants.k_isEnabled) {
      return Commands.run(
          () -> {
            stop();
          }, this);
    } else {
      return Commands.none();
    }
  }

}