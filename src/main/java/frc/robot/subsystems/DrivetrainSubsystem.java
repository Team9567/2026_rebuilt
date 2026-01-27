// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveTrainConstants;

public class DrivetrainSubsystem extends SubsystemBase {
  SparkFlex leftFrontMotor = new SparkFlex(DriveTrainConstants.kLeftFrontMotorCanID, MotorType.kBrushless);
  SparkFlex leftBackMotor = new SparkFlex(DriveTrainConstants.kLeftBackMotorCanID, MotorType.kBrushless);
  SparkFlex rightFrontMotor = new SparkFlex(DriveTrainConstants.kRightFrontMotorCanID, MotorType.kBrushless);
  SparkFlex rightBackMotor = new SparkFlex(DriveTrainConstants.kRightBackMotorCanID, MotorType.kBrushless);
  DifferentialDrive drivetrain;
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Meters.of(DriveTrainConstants.kTrackWidth));
  DifferentialDrivePoseEstimator odometry;
  AHRS m_gyro;
  Field2d field = new Field2d();

  double targetMeters = 0;
  double targetAngle = 0;
  PIDController drivepid = new PIDController(DriveTrainConstants.kDriveP, DriveTrainConstants.kDriveI,
      DriveTrainConstants.kDriveD);
  PIDController turnpid = new PIDController(DriveTrainConstants.kTurnP, DriveTrainConstants.kTurnI,
      DriveTrainConstants.kTurnD);

  Optional<Trigger> lowGearTrigger;

  /** Creates a new ExampleSubsystem. */
  public DrivetrainSubsystem() {
    turnpid.setTolerance(1);
    m_gyro = new AHRS(DriveTrainConstants.kGyroPort);
    m_gyro.reset();
    odometry = new DifferentialDrivePoseEstimator(
        kinematics,
        m_gyro.getRotation2d(),
        getLeftEncoder(),
        getRightEncoder(),
        new Pose2d(0, 0, new Rotation2d()));

    SmartDashboard.putNumber("drivetrain/wheelconversion", DriveTrainConstants.kPositionConversionFactor);

    for (SparkFlex motor : new SparkFlex[] {
        leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor }) {
      SparkMaxConfig config = new SparkMaxConfig();
      config
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(80);
      config.softLimit
          .forwardSoftLimitEnabled(false)
          .reverseSoftLimitEnabled(false);
      config.openLoopRampRate(DriveTrainConstants.kMotorRampTime);
      config.encoder.positionConversionFactor(DriveTrainConstants.kPositionConversionFactor);
      motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    SparkFlexConfig leftbackConfig = new SparkFlexConfig();
    leftbackConfig.follow(leftFrontMotor);
    leftbackConfig.inverted(false);
    leftBackMotor.configure(leftbackConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    SparkFlexConfig rightbackConfig = new SparkFlexConfig();
    rightbackConfig.follow(rightFrontMotor);
    rightbackConfig.inverted(false);
    rightBackMotor.configure(rightbackConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig leftfrontConfig = new SparkFlexConfig();
    leftfrontConfig.inverted(false);
    leftFrontMotor.configure(leftfrontConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig rightfrontConfig = new SparkFlexConfig();
    rightfrontConfig.inverted(true);
    rightFrontMotor.configure(rightfrontConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    drivetrain = new DifferentialDrive(leftFrontMotor, rightFrontMotor);
  }

  public void setGearTrigger(Trigger t) {
    lowGearTrigger = Optional.of(t);
    // Sets t to a trigger and sending it to RobotContainer
  }

  public void arcadeDrive(double speed, double turn) {
    if (lowGearTrigger.isPresent()) {
      if (lowGearTrigger.get().getAsBoolean()) {
        speed /= 4;
        turn /= 4;
      }
    }
    drivetrain.arcadeDrive(speed, turn);
    // The mathematics for the high/low gear and ArcadeDrive
  }

  // Same as arcadeDrive, but applies no speed adjustments (low gear)
  public void arcadeDriveRaw(double speed, double turn) {
    drivetrain.arcadeDrive(speed, turn);
  }

  public double getLeftEncoder() {
    double encoder = leftFrontMotor.getEncoder().getPosition();
    return encoder;
  }

  public double getRightEncoder() {
    double encoder = rightBackMotor.getEncoder().getPosition();
    return encoder;
  }

  public Pose2d getEstimatedPosition() {
    return odometry.getEstimatedPosition();
  }

  public double getAverageTicks() {
    return (leftFrontMotor.getEncoder().getPosition() + rightBackMotor.getEncoder().getPosition()) / 2;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(m_gyro.getRotation2d(), new DifferentialDriveWheelPositions(getLeftEncoder(), getRightEncoder()));
    Pose2d bot = odometry.getEstimatedPosition();
    field.setRobotPose(bot);

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public Command driveCommand(double distance) {
    return startRun(() -> {
      double avgPos = getAverageTicks();
      targetMeters = avgPos + distance;

      drivepid.reset();
      SmartDashboard.putNumber("drivetrain/pid/targetDistance", targetMeters);

    }, () -> {
      double avgPos = getAverageTicks();
      double speed = drivepid.calculate(avgPos, targetMeters);
      speed = MathUtil.clamp(speed, -0.25, 0.25);
      arcadeDrive(speed, 0);
      SmartDashboard.putNumber("drivetrain/pid/position", avgPos);
    }).until(() -> {
      return drivepid.atSetpoint();
    });
  }

  public Command turnCommand(double angle) {
    return startRun(() -> {
      double curAngle = m_gyro.getAngle();
      targetAngle = curAngle + angle;

      turnpid.reset();
      SmartDashboard.putNumber("drivetrain/pid/targetAngle", targetAngle);
    }, () -> {
      double rotation = -turnpid.calculate(m_gyro.getAngle(), angle);
      rotation = MathUtil.clamp(rotation, -0.25, 0.25);
      SmartDashboard.putNumber("drivetrain/pid/orientation", m_gyro.getAngle());
      SmartDashboard.putNumber("drivetrain/pid/rotation", rotation);
      arcadeDrive(0, rotation);
    }).until(() -> {
      return turnpid.atSetpoint();
    });
  }

}
