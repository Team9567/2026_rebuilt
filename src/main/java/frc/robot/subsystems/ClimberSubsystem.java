package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FuelConstants;

/**
 * Elevator subsystem using SparkMax with Vortex motor
 */
@Logged(name = "ClimberSubsystem")
public class ClimberSubsystem extends SubsystemBase {

  // Constants
  private static boolean kIsEnabled = false;
  private final int canID = 9; // placeholder
  private final double gearRatio = 56.0 / 45.0;
  private final double kP = 0; // placeholder
  private final double kI = 0;
  private final double kD = 0;
  private final double kS = 0.145;
  private final double kV = 0.0021139;
  private final double kA = 0;
  private final double kG = 0;
  private final double maxVelocity = 1; // placeholder, meters per second
  private final double maxAcceleration = 1; // placeholder, meters per second squared
  private final boolean brakeMode = true;
  private final boolean enableStatorLimit = true;
  private final int statorCurrentLimit = 40;
  private final boolean enableSupplyLimit = false;
  private final double supplyCurrentLimit = 40; // placeholder
  private final double maxClimbHeightTicks = 127;
 
  private final double maxHeightTicks = 99.5; // How high the climber is at its peak (not
                                                                          // over limit)
  private final double minHeightTicks = 0;
  private final double hangHeightTicks = 30;
  private final double kHomingAmpsThresh = 4; // Used to be 20 amps
  private final double climberOffsetMeters = 0.15; // placeholder

  private double m_ampStorage[] = { 0, 0, 0, 0 };
  private int m_indexAmps = 0;

  // Motor controller
  private SparkMax motor;
  private RelativeEncoder encoder;
  private SparkClosedLoopController sparkPidController;
  private DoubleSupplier m_zSupplier;

  private boolean m_isHomed = false;

  /**
   * Creates a new Elevator Subsystem.
   */
  public ClimberSubsystem() {
    if (kIsEnabled) {
      // Initialize motor controller
      SparkMaxConfig motorConfig = new SparkMaxConfig();
      motor = new SparkMax(canID, MotorType.kBrushless);
      motorConfig.idleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);

      // Configure encoder
      encoder = motor.getEncoder();
      encoder.setPosition(0);

      // Set ramp rates

      // Set current limits
      motorConfig.smartCurrentLimit(statorCurrentLimit);
      // motorConfig.inverted(true);

      // Configure Feedback and Feedforward
      sparkPidController = motor.getClosedLoopController();
      motorConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(kP, kI, kD, ClosedLoopSlot.kSlot0);
      motorConfig.closedLoop.feedForward.kS(kS).kV(kV).kA(kA);
      motorConfig.closedLoop.feedForward.kG(kG);

      motorConfig.closedLoop.maxMotion
          .cruiseVelocity(1000)
          .maxAcceleration(5000)
          .allowedProfileError(2);

      // Configure Encoder Gear Ratio
      // motorConfig.encoder
      // .positionConversionFactor(1 / gearRatio)
      // .velocityConversionFactor((1 / gearRatio) / 60); // Covnert RPM to RPS

      motorConfig.softLimit
          .forwardSoftLimitEnabled(false)
          .forwardSoftLimit(maxClimbHeightTicks)
          .reverseSoftLimitEnabled(false)
          .reverseSoftLimit(minHeightTicks);
      // Save configuration
      motor.configure(
          motorConfig,
          ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);
    }

    setDefaultCommand(homeCommand()); // changed from stopCommand
  }

  public void setZSupplier(DoubleSupplier supplier) {
    m_zSupplier = supplier;
  }

  @SuppressWarnings("unused")
  private double getZAxis() {
    return m_zSupplier.getAsDouble();
  }

  public Command moveUp() {
    return run(() -> {
      motor.set(0.35);
    });
  }

  public Command moveDown() {
    return run(() -> {
      motor.set(-0.25);
    });
  }

  /**
   * Update simulation and telemetry.
   */
  @Override
  public void periodic() {
    if (kIsEnabled) {
      SmartDashboard.putNumber("Climber/Encoder position", motor.getEncoder().getPosition());
      SmartDashboard.putNumber("Climber/set point", motor.getClosedLoopController().getSetpoint());
      SmartDashboard.putBoolean("Climber/is at setpoint", motor.getClosedLoopController().isAtSetpoint());
      SmartDashboard.putNumber("Climber/Output Current", motor.getOutputCurrent());
      SmartDashboard.putNumber("Climber/Voltage", getVoltage());
      SmartDashboard.putBoolean("Climber/Is Homed (ON START NOT CURRENTLY)", this.m_isHomed);
    }

    if (this.getCurrentCommand() != null) {
      if (this.getCurrentCommand() == this.getDefaultCommand()) {
        SmartDashboard.putString("Climber - Current Command", "Idle?");
      } else {
        String cClimberCommandName = this.getCurrentCommand().getName();
        SmartDashboard.putString("Climber - Current Command", cClimberCommandName);
      }
    }
  }

  /**
   * Update simulation.
   */
  @Override
  public void simulationPeriodic() {

  }

  /**
   * Get the current position in the Rotations.
   * 
   * @return Position in Rotations
   */
  @Logged(name = "Position/Rotations")
  public double getPosition() {
    if (kIsEnabled) {
      // Rotations
      return encoder.getPosition() / gearRatio;
    }
    return 0;
  }

  /**
   * Get the current velocity in rotations per second.
   * 
   * @return Velocity in rotations per second
   */
  @Logged(name = "Velocity")
  public double getVelocity() {
    if (kIsEnabled) {
      return encoder.getVelocity() / gearRatio / 60.0; // Convert from RPM to RPS
    }
    return -9567;
  }

  /**
   * Get the current applied voltage.
   * 
   * @return Applied voltage
   */
  @Logged(name = "Voltage")
  public double getVoltage() {
    if (kIsEnabled) {
      return motor.getAppliedOutput() * motor.getBusVoltage();
    }
    return -9567;
  }

  /**
   * Get the current motor current.
   * 
   * @return Motor current in amps
   */
  public double getCurrent() {
    if (kIsEnabled) {
      return motor.getOutputCurrent();
    }
    return -9567;
  }

  /**
   * Get the current motor temperature.
   * 
   * @return Motor temperature in Celsius
   */
  public double getTemperature() {
    if (kIsEnabled) {
      return motor.getMotorTemperature();
    }
    return -9567;
  }

  /**
   * Set elevator position.
   * 
   * @param position The target position in meters
   */
  public Command setPositionCommand(double position) {
    return run(() -> {
      setPosition(position);
    });// .until(() -> motor.getClosedLoopController().isAtSetpoint()).withName("Set
       // Position");
  }

  /**
   * Set elevator position with acceleration.
   * 
   * @param position     The target position in meters
   * @param acceleration The acceleration in meters per second squared
   */
  public void setPosition(double position) {
    if (kIsEnabled) {
      if (m_isHomed) {
        sparkPidController.setSetpoint(
            position,
            ControlType.kMAXMotionPositionControl);
      }
    }
  }

  /**
   * Set elevator velocity.
   * 
   * @param velocity The target velocity in meters per second
   */
  public void setVelocity(double velocity) {
    setVelocity(velocity, 0);
  }

  /**
   * Set elevator velocity with acceleration.
   * 
   * @param velocity     The target velocity in meters per second
   * @param acceleration The acceleration in meters per second squared
   */
  public void setVelocity(double velocity, double acceleration) {
    if (kIsEnabled) {
      sparkPidController.setSetpoint(
          velocity,
          ControlType.kVelocity,
          ClosedLoopSlot.kSlot0);
    }
  }

  /**
   * Set motor voltage directly.
   * 
   * @param voltage The voltage to apply
   */
  public void setVoltage(double voltage) {
    if (kIsEnabled) {
      motor.setVoltage(voltage);
    }
  }

  /**
   * Creates a command to set the elevator to a specific height.
   * 
   * @param heightMeters The target height in meters
   * @return A command that sets the elevator to the specified height
   */
  public Command setHeightCommand(double heightMeters) {
    if (m_isHomed) {
      return runOnce(() -> setPosition(heightMeters));
    }
    return Commands.none();
  }

  /**
   * Creates a command to move the elevator to a specific height with a profile.
   * 
   * @param heightMeters The target height in meters
   * @return A command that moves the elevator to the specified height
   *
   */

  public Command moveToHeightCommand(double heightMeters) {
    if (kIsEnabled) {
      if (m_isHomed) {
        return run(() -> {
          double currentHeight = getPosition() + climberOffsetMeters;
          double error = heightMeters - currentHeight;
          double velocity = Math.signum(error) * Math.min(Math.abs(error) * 2.0, maxVelocity);
          setVelocity(velocity);
        }).until(() -> {
          double currentHeight = getPosition() + climberOffsetMeters;
          return Math.abs(heightMeters - currentHeight) < 0.02; // 2cm tolerance
        }).withName("Move to height Command");
      }
    }
    return Commands.none();
  }

  public boolean hasReachedHardstop() {
    double currentAverage = (m_ampStorage[0] + m_ampStorage[1] + m_ampStorage[2] + m_ampStorage[3]) / 4;

    return currentAverage > kHomingAmpsThresh;
  }

  public Command homeCommand() {
    return run(() -> {
      if (kIsEnabled) {
        if (!m_isHomed) {
          double currentAmps = motor.getOutputCurrent();
          m_ampStorage[m_indexAmps++ % m_ampStorage.length] = currentAmps;
          if (hasReachedHardstop()) {
            setVoltage(0);
            encoder.setPosition(0);
            m_isHomed = true;
            SparkMaxConfig config = new SparkMaxConfig();
            config.softLimit.reverseSoftLimitEnabled(true);
            motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
          } else {
            motor.set(-0.05); //Used to be -0.15
          }
        }
      }
    }).until(() -> m_isHomed).withName("Home Command");
  }

  public Command hangCommand() {
    return run(() -> {
      setPosition(hangHeightTicks);
    }).withName("Hang Command");
  }

  public Command startClimbCommand() {
    return run(() -> {
      setPosition(maxHeightTicks);
    }).withName("Start Climb Command");
  }

  public Command climbCommand() {
    return startClimbCommand()
        .until(() -> motor.getClosedLoopController().isAtSetpoint())
        .andThen(hangCommand())
        .until(() -> motor.getClosedLoopController().isAtSetpoint());
  }

  public Command unclimbCommand() {
    return run(() -> {
      setPosition(maxHeightTicks);
    }).withName("Unclimb Command");
  }

  /**
   * Creates a command to stop the elevator.
   * 
   * @return A command that stops the elevator
   */
  public Command stopCommand() {
    return runOnce(() -> setVelocity(0)).withName("Stop Command");
  }

  /**
   * *
   * 
   * @param velocityMetersPerSecond The target velocity in meters per second
   * @return A command that moves the elevator at the specified velocity
   */
  public Command moveAtVelocityCommand(double velocityMetersPerSecond) {
    return run(() -> setVelocity(velocityMetersPerSecond));
  }

}
