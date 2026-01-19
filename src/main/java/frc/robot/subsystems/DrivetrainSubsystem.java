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

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
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
  DifferentialDrivePoseEstimator odometry;
  private AHRS m_gyro;
  Field2d field = new Field2d();

  Optional<Trigger> lowGearTrigger;

  /** Creates a new ExampleSubsystem. */
  public DrivetrainSubsystem() {
    double wheelConversion = Inches.of(DriveTrainConstants.kWheelCircumference * DriveTrainConstants.kGearRatio)
        .in(Meters);
    SmartDashboard.putNumber("drivetrain/wheelconversion", wheelConversion);

    try (AHRS m_gyro = new AHRS(DriveTrainConstants.kGyroPort)) {
      m_gyro.reset();
    }

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

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  public void arcadeDrive(double aSpeed, double aRotation) {
    drivetrain.arcadeDrive(aSpeed, aRotation);
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
}
