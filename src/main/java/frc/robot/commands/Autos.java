// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FuelSubsystem;

public final class Autos {

  public enum Handed {
    Left,
    Right
  };

  public enum Closeness {
    Near,
    Far
  };

  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(DrivetrainSubsystem subsystem) {
    return Commands.sequence(subsystem.driveCommand(2.0),
        new DriveToPointCommand(new Pose2d(2.0, 2.0, new Rotation2d()), subsystem));
  }

  public static Command smartShoot(FuelSubsystem shooter, DrivetrainSubsystem drivetrain) {
    return shooter.smartShoot(() -> {
      return drivetrain.getDistanceToHub();
    });
  }

  public static Command justShoot(FuelSubsystem shooter) {
    return shooter.shootCommand();
  }

  public static Command smartClimb(ClimberSubsystem climber, DrivetrainSubsystem drivetrain, Handed handed,
      Closeness closeness) {
    double approachAngle = 0;
    double slotDistance = 0;
    var maybeAlliance = DriverStation.getAlliance();
    var alliance = maybeAlliance.isEmpty() ? Alliance.Blue : maybeAlliance.get();

    if ((closeness == Closeness.Near && handed == Handed.Right)
        || (closeness == Closeness.Far && handed == Handed.Left)) {
      slotDistance = 0.5;
    } else {
      slotDistance = -0.5;
    }
    if ((closeness == Closeness.Near && alliance == Alliance.Blue)
        || (closeness == Closeness.Far && alliance == Alliance.Red)) {
      approachAngle = Math.PI + Math.PI / 4;
    } else {
      approachAngle = Math.PI / 4;
    }

    Pose2d currentPose2d = drivetrain.getEstimatedPosition();
    Pose2d newPose2d = new Pose2d(currentPose2d.getTranslation(), new Rotation2d(approachAngle));
    return climber.startClimbCommand()
        .andThen(new DriveToPointCommand(newPose2d, drivetrain))
        .andThen(drivetrain.driveCommand(Feet.of(-4.24).in(Meters)))
        .andThen(drivetrain.turnCommand(45))
        .andThen(drivetrain.driveCommand(slotDistance))
        .andThen(climber.hangCommand());
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
