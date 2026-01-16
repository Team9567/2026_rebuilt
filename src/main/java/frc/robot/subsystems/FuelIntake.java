package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FuelIntakeConstants;

public class FuelIntake extends SubsystemBase {
  private SparkMax fuelIntakeMotor;

  public FuelIntake() {
    if (FuelIntakeConstants.k_isEnabled) {
      // Set up the Intake motor as a brushless motor
      fuelIntakeMotor = new SparkMax(FuelIntakeConstants.kIntakeMotorID, MotorType.kBrushless);

      // Create and apply configuration for intake motor. Voltage compensation helps
      // the intake behave the same as the battery
      // voltage dips. The current limit helps prevent breaker trips or burning out
      // the motor in the event the intake stalls.
      SparkMaxConfig config = new SparkMaxConfig();
      config
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(FuelIntakeConstants.kIntakeMotorCurrentLimit)
          .voltageCompensation(FuelIntakeConstants.kIntakeMotorVoltageCompens)
          .inverted(true);
      config.softLimit
          .forwardSoftLimitEnabled(false)
          .reverseSoftLimitEnabled(false);
      config.encoder.positionConversionFactor(FuelIntakeConstants.kIntakeGearRatio);
      config.encoder.velocityConversionFactor(1.0 / 60.0);
      fuelIntakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      fuelIntakeMotor.setCANTimeout(250);
    }
  }

  public void runIntakeRaw(double speed) {
    fuelIntakeMotor.set(speed);
    SmartDashboard.putNumber("intake/speed", speed);
  }

  public Command runIntakeRawCommand(double speed) {
    if (FuelIntakeConstants.k_isEnabled) {
      return Commands.run(
          () -> {
            runIntakeRaw(speed);
          }, this);
    } else {
      return Commands.none();
    }
  }
}
