// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Translation2d;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

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
    public static final boolean kIsEnabled = true;
    public static final boolean kIsLimelightEnabled = false;
    public static final String kLimelightNetworkName = "limelight";
    public static final int kLeftFrontMotorCanID = 1;
    public static final int kLeftBackMotorCanID = 3;
    public static final int kRightFrontMotorCanID = 2;
    public static final int kRightBackMotorCanID = 4;
    public static final double kWheelDiameter = 6;
    public static final double kWheelCircumference = kWheelDiameter * Math.PI;
    public static final double kGearRatio = 8.46;
    public static final double kMotorRampTime = 0.1;
    public static final double kWheelDiameterInches = 6.17;
    public static final double kTrackWidth = 0.546; // 21.5 inches in meters, measured midpoint center wheel to midpoint
                                                    // center wheel

    public static final double kPositionConversionFactor = Inches.of(kWheelCircumference / kGearRatio)
        .in(Meters);

    public static final NavXComType kGyroPort = NavXComType.kMXP_SPI;
    public static final double kDriveP = 6.0;
    public static final double kDriveI = 0;
    public static final double kDriveD = 0;
    public static final double kTurnP = 0.125;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0.005;

    public static final int kBlueHubBlueSideTagID = 26;
    public static final int kBlueHubRedSideTagID = 20;
    public static final int kRedHubRedSideTagID = 10;
    public static final int kRedHubBlueSideTagID = 4;

    public static final Translation2d kBlueHubCoord = new Translation2d(Inches.of(181.56).in(Meters),
        Inches.of(158.32).in(Meters));
    public static final Translation2d kRedHubCoord = new Translation2d(Inches.of(650.12-181.56).in(Meters),
        Inches.of(158.32).in(Meters));
  }

}
