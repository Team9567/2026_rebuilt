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
import frc.robot.Constants.FuelShooterConstants;

public class FuelShooter extends SubsystemBase {
  private SparkMax fuelShooterMotor;

  public FuelShooter() {
    if (FuelShooterConstants.k_isEnabled) {
      // Set up the shooter motor as a brushless motor
      fuelShooterMotor = new SparkMax(FuelShooterConstants.kShooterMotorID, MotorType.kBrushless);

      // Create and apply configuration for shooter motor. Voltage compensation helps
      // the shooter behave the same as the battery
      // voltage dips. The current limit helps prevent breaker trips or burning out
      // the motor in the event the shooter stalls.
      SparkMaxConfig config = new SparkMaxConfig();
      config
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(FuelShooterConstants.kShooterMotorCurrentLimit)
          .voltageCompensation(FuelShooterConstants.kShooterMotorVoltageCompens)
          .inverted(true);
      config.softLimit
          .forwardSoftLimitEnabled(false)
          .reverseSoftLimitEnabled(false);
      config.encoder.positionConversionFactor(FuelShooterConstants.kShooterGearRatio);
      config.encoder.velocityConversionFactor(1.0 / 60.0);
      fuelShooterMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      fuelShooterMotor.setCANTimeout(250);
    }
  }

  public void runShooterRaw(double speed) {
    fuelShooterMotor.set(speed);
    SmartDashboard.putNumber("shooter/speed", speed);
  }

  public Command runShooterRawCommand(double speed) {
    if (FuelShooterConstants.k_isEnabled) {
      return Commands.run(
          () -> {
            runShooterRaw(speed);
          }, this);
    } else {
      return Commands.none();
    }
  }
}
