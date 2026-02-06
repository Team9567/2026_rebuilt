package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.FuelConstants;

public class FuelSubsystem extends SubsystemBase {
  private SparkMax fuelShooterMotor;
  private SparkMax fuelIntakeMotor;
  SimpleMotorFeedforward shooterFeedforward = new SimpleMotorFeedforward(FuelConstants.kShooterFeedForwardStatic,
      FuelConstants.kShooterFeedForwardVelocity);
  PIDController shooterController = new PIDController(FuelConstants.kShooterP, FuelConstants.kShooterI,
      FuelConstants.kShooterD);
  private double m_dashboardShooterRPS;

  public FuelSubsystem() {
    if (FuelConstants.k_isEnabled) {
      // Set up the shooter motor as a brushless motor
      fuelShooterMotor = new SparkMax(FuelConstants.kShooterMotorID, MotorType.kBrushless);
      // Set up the Intake motor as a brushless motor
      fuelIntakeMotor = new SparkMax(FuelConstants.kIntakeMotorID, MotorType.kBrushless);

      /**
       * Create and apply configuration for shooter motor. Voltage compensation helps
       * the shooter behave the same as the battery
       * voltage dips. The current limit helps prevent breaker trips or burning out
       * the motor in the event the shooter stalls.
       */
      SparkMaxConfig shooterConfig = new SparkMaxConfig();
      shooterConfig
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(FuelConstants.kShooterMotorCurrentLimit)
          .voltageCompensation(FuelConstants.kShooterMotorVoltageCompens)
          .inverted(false);
      shooterConfig.softLimit
          .forwardSoftLimitEnabled(false)
          .reverseSoftLimitEnabled(false);
      shooterConfig.encoder.positionConversionFactor(FuelConstants.kShooterGearRatio);
      shooterConfig.encoder.velocityConversionFactor(1.0 / 60.0);

      shooterConfig.closedLoop
          .p(FuelConstants.kShooterP)
          .i(FuelConstants.kShooterI)
          .d(FuelConstants.kShooterD)

              .maxMotion
          .cruiseVelocity(0)
          .maxAcceleration(FuelConstants.kMaxAcceleration)
          .allowedProfileError(FuelConstants.kProfileErrorRPS);

      shooterConfig.closedLoop.feedForward
          .kS(FuelConstants.kShooterFeedForwardStatic)
          .kV(FuelConstants.kShooterFeedForwardVelocity)
          .kA(FuelConstants.kShooterFeedForwardAccel);
      fuelShooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      fuelShooterMotor.setCANTimeout(250);

      /**
       * Create and apply configuration for intake motor. Voltage compensation helps
       * the intake behave the same as the battery
       * voltage dips. The current limit helps prevent breaker trips or burning out
       * the motor in the event the intake stalls.
       */
      SparkMaxConfig intakeConfig = new SparkMaxConfig();
      intakeConfig
          .idleMode(IdleMode.kCoast)
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

      // Pass in stop when needed to be stopped.
      setDefaultCommand(stopCommand());
    }
  }

  // The following code is what will set our speed, voltage, and our stop control.
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

  public void setShooterVelocity(double rps) {
    SparkClosedLoopController controller = fuelShooterMotor.getClosedLoopController();
    controller.setSetpoint(rps, ControlType.kMAXMotionVelocityControl);
  }

  public void setIntakeVelocity(double speed) {
    fuelIntakeMotor.set(speed);
  }

  /**
   * The following code is what sets our four actions for our robot:
   * Intake, Eject, Spinup, and Shoot.
   * Intake is how the robot sucks up Fuel.
   * Eject is how the robot releases Fuel without Shooting it.
   * Spinup is the action before Shoot, where the ShooterMotor reaches full power
   * before Shoot.
   * Shoot is how the robot will launch Fuel.
   */
  public Command intakeCommand() {
    if (FuelConstants.k_isEnabled) {
      return setCommand(FuelConstants.kIntakeShooterMotorSpeed, FuelConstants.kIntakeIntakeMotorSpeed);
    } else {
      return Commands.none();
    }
  }

  public Command ejectCommand() {
    if (FuelConstants.k_isEnabled) {
      return setCommand(FuelConstants.kEjectShooterMotorSpeed, FuelConstants.kEjectIntakeMotorSpeed);
    } else {
      return Commands.none();
    }
  }

  public Command spinupCommand() {
    if (FuelConstants.k_isEnabled) {
      return setCommand(FuelConstants.kSpinupShooterMotorSpeed, FuelConstants.kSpinupIntakeMotorSpeed);
    } else {
      return Commands.none();
    }
  }

  public Command shootCommand() {
    if (FuelConstants.k_isEnabled) {
      return setCommand(FuelConstants.kShootShooterMotorSpeed, FuelConstants.kShootIntakeMotorSpeed);
    } else {
      return Commands.none();
    }
  }

  public Command testCommand() {
    if (FuelConstants.k_isEnabled) {
      return setCommand(0.5, 0.5);
    } else {
      return Commands.none();
    }
  }

  public Command shootVelocityCommand(double rps) {
    if (FuelConstants.k_isEnabled) {
      return run(() -> {
        setShooterVelocity(rps);
        setIntakeVelocity(FuelConstants.kShootIntakeMotorSpeed);
      });
    } else {
      return Commands.none();
    }
  }

  public Command shootDashboardVelocityCommand() {
    if (FuelConstants.k_isEnabled) {
      return run(() -> {
        setShooterVelocity(m_dashboardShooterRPS);
        setIntakeVelocity(FuelConstants.kShootIntakeMotorSpeed);
      });
    } else {
      return Commands.none();
    }
  }

  @Override
  public void periodic() {
    m_dashboardShooterRPS = SmartDashboard.getNumber("shooter/rps", -9567);
  }
}