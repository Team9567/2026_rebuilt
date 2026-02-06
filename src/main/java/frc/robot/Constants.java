// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.units.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    public static final int kDriverControllerA = 1;
    public static final int kDriverControllerB = 2;
    public static final int kDriverControllerX = 3;
    public static final int kDriverControllerY = 4;

    public static final int kDriverControllerLeftBumper = 5;
    public static final int kDriverControllerRightBumper = 6;
    public static final int kDriverControllerLeftTrigger = 2;
    public static final int kDriverControllerRightTrigger = 3;

    public static final int kDriverControllerBack = 7;
    public static final int kDriverControllerStart = 8;
    public static final int kDriverControllerLeftJoystickButton = 9;
    public static final int kDriverControllerRightJoystickButton = 10;

    public static final int kControllerLeftVertical = 1;
    public static final int kControllerLeftHorizontal = 0;
    public static final int kControllerRightVertical = 5;
    public static final int kControllerRightHorizontal = 4;

    public static final int kDriverControllerPOVLeft = 270;
    public static final int kDriverControllerPOVRight = 90;
    public static final int kDriverControllerPOVUp = 0;
    public static final int kDriverControllerPOVDown = 180;
  }

  public static class DriveTrainConstants {
    public static final int kLeftFrontMotorCanID = 1;
    public static final int kLeftBackMotorCanID = 3;
    public static final int kRightFrontMotorCanID = 2;
    public static final int kRightBackMotorCanID = 4;
    public static final double kWheelDiameter = 6;
    public static final double kWheelCircumference = kWheelDiameter * Math.PI;
    public static final double kGearRatio = 8.46;
    public static final double kMotorRampTime = 0.1;
    public static final double kWheelDiameterInches = 6.17;

    // 21.5 inches in meters, measured midpoint center wheel to midpoint center
    // wheel
    public static final double kTrackWidth = 0.546;

    public static final double kPositionConversionFactor = Units.Inches.of(kWheelCircumference / kGearRatio)
        .in(Units.Meters);

    public static final NavXComType kGyroPort = NavXComType.kMXP_SPI;
    public static final double kDriveP = 6.0;
    public static final double kDriveI = 0;
    public static final double kDriveD = 0;
    public static final double kTurnP = 0.125;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0.005;
  }

  public static class FuelConstants {
    public static final boolean k_isEnabled = true;
    public static final int kShooterMotorID = 5;
    public static final double kShooterGearRatio = 0.5;
    public static final double kShooterMotorVoltageCompens = 10;
    public static final int kShooterMotorCurrentLimit = 60;

    public static final int kIntakeMotorID = 6;
    public static final double kIntakeGearRatio = 0.5;
    public static final double kIntakeMotorVoltageCompens = 10;
    public static final int kIntakeMotorCurrentLimit = 60;

    public static final double kShooterFeedForwardStatic = 0;
    public static final double kShooterFeedForwardVelocity = 0;
    public static final double kShooterFeedForwardAccel = 0;
    public static final double kMaxAcceleration = 10;
    public static final double kProfileErrorRPS = 80;
    public static final double kShooterP = 0;
    public static final double kShooterI = 0;
    public static final double kShooterD = 0;

    /*
     * Clockwise = positive number
     * Counterclockwise = negative number
     */
    public static final double kIntakeIntakeMotorSpeed = -0.30; //used to be counter-clockwise
    public static final double kIntakeShooterMotorSpeed = 0.30; 
    public static final double kEjectIntakeMotorSpeed = 0.30;
    public static final double kEjectShooterMotorSpeed = -0.30;
    public static final double kSpinupIntakeMotorSpeed = 0;
    public static final double kSpinupShooterMotorSpeed = 0.30;
    public static final double kShootIntakeMotorSpeed = 0.30;
    public static final double kShootShooterMotorSpeed = 0.30;

    public static final double kShootShooterMotorVelocity = 0;
    public static final double kShootIntakeMotorVelocity = 0;
  }
}