// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

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
        // The mathematics for the high/low gear and ArcadeDrive
        speed /= 2;
        turn /= 2;
      }
    }

    if (reverseGearTrigger.isPresent()) {
      if (reverseGearTrigger.get().getAsBoolean()) {
        // The mathematics for the reverse gear and ArcadeDrive
        speed *= -1;
        turn *= -1;
      }
    }

    arcadeDriveRaw(speed, turn);
  }


  // Same as arcadeDrive, but applies no speed adjustments (low gear)
  public void arcadeDriveRaw(double speed, double turn) {
    if (DriveTrainConstants.kIsEnabled) {
      drivetrain.arcadeDrive(speed, turn);
    }
  }

  public double getLeftEncoder() {
    double encoder = leftFrontMotor.getEncoder().getPosition();
    return encoder;
  }

  public double getRightEncoder() {
    double encoder = rightBackMotor.getEncoder().getPosition();
    return encoder;
  }

  public float getGyroZValue() {
    float altitude = m_gyro.getAltitude();
    return altitude;
  }

  public Pose2d getEstimatedPosition() {
    return odometry.getEstimatedPosition();
  }

  public double getAverageTicks() {
    return (leftFrontMotor.getEncoder().getPosition() + rightBackMotor.getEncoder().getPosition()) / 2;
  }

  public double getAngleToTarget(Pose2d target) {
    var pose = getEstimatedPosition();
    var theta = Math.atan((target.getY() - pose.getY()) / (target.getX() - pose.getX()));
    var degrees = theta * (Math.PI / 180);
    return degrees;
  }

  public double getDistanceToTarget(Pose2d target) {
    var pose = getEstimatedPosition();
    var distance = pose.getTranslation().minus(target.getTranslation()).getNorm();
    return distance;
  }

  public double getAngleToHub() {
    double angle = getAngleToTarget(new Pose2d(getAllianceHub(), new Rotation2d()));
    SmartDashboard.putNumber("drivetrain/hub angle", angle);
    return angle;
  }

  public double getDistanceToHub() {
    double distance = getDistanceToTarget(new Pose2d(getAllianceHub(), new Rotation2d()));
    SmartDashboard.putNumber("drivetrain/hub distance", distance);
    return distance;
  }

  public Translation2d getAllianceHub() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isEmpty() || alliance.get() == Alliance.Blue) {
      return Constants.DriveTrainConstants.kBlueHubCoord;
    } else {
      return Constants.DriveTrainConstants.kRedHubCoord;
    }
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    // updates odometry
    odometry.update(m_gyro.getRotation2d(), new DifferentialDriveWheelPositions(getLeftEncoder(), getRightEncoder()));
    if (DriveTrainConstants.kIsEnabled) {
      // updates position based on visible april tags
      LimelightHelpers.SetRobotOrientation(DriveTrainConstants.kLimelightNetworkName,
          odometry.getEstimatedPosition().getRotation().getDegrees(),
          0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers
          .getBotPoseEstimate_wpiBlue_MegaTag2(DriveTrainConstants.kLimelightNetworkName);

      boolean doRejectUpdate = false;

      // if our angular velocity is greater than 360 degrees per second, ignore vision updates
      if (Math.abs(m_gyro.getRate()) > 360) {
        doRejectUpdate = true;
      }
      if (mt2.tagCount == 0) {
        doRejectUpdate = true;
      }
      if (!doRejectUpdate) {
        odometry.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
        odometry.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      }
    }

    // sets robot's positon on field based on gyro and limelight
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
      arcadeDriveRaw(speed, 0);
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
      arcadeDriveRaw(0, rotation);
    }).until(() -> {
      return turnpid.atSetpoint();
    });
  }

}
