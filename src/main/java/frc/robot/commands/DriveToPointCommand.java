// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;

/** An example command that uses an example subsystem. */
public class DriveToPointCommand extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final DrivetrainSubsystem m_subsystem;
  private Pose2d m_targetPose;
  PIDController drivepid = new PIDController(7.5, 0, 0.15);
  PIDController turnpid = new PIDController(0.1, 0, 0.01);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveToPointCommand(Pose2d target, DrivetrainSubsystem subsystem) {
    m_subsystem = subsystem;
    m_targetPose = target;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivepid.setTolerance(0.05);
    turnpid.setTolerance(3);
    var pose = m_subsystem.getEstimatedPosition();
    var theta = Math.atan((m_targetPose.getY()-pose.getY())/(m_targetPose.getX()-pose.getX()));
    var degrees = theta*(Math.PI/180);
      m_targetPose = new Pose2d(m_targetPose.getX(),m_targetPose.getY(),new Rotation2d(degrees));
      SmartDashboard.putNumber("drive to point/target pose/x", m_targetPose.getX());
      SmartDashboard.putNumber("drive to point/target pose/y", m_targetPose.getY());
      SmartDashboard.putNumber("drive to point/target pose/rotation", m_targetPose.getRotation().getDegrees());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var pose = m_subsystem.getEstimatedPosition();
    var distance = pose.getTranslation().minus(m_targetPose.getTranslation()).getNorm();
    var robotRotation = m_targetPose.relativeTo(pose).getTranslation().getAngle().getDegrees();

    // Adjusting distance and rotation to allow driving backwards to a point behind the robot.
    if (Math.abs(robotRotation) > 100) {
      distance *= -1;
      double angleSign = Math.signum(robotRotation);
      robotRotation += 180 * angleSign * -1;
    }

    double speed = 0;
    if (Math.abs(robotRotation) < 15) {
     speed = drivepid.calculate(0, distance);
     speed = MathUtil.clamp(speed, -0.30, 0.30);
    }

    double rotation = turnpid.calculate(0, robotRotation);
    rotation = MathUtil.clamp(rotation, -0.25, 0.25);
    SmartDashboard.putNumber("drive to point/speed", speed);
    SmartDashboard.putNumber("drive to point/distance", distance);
    SmartDashboard.putNumber("drive to point/robot rotation", robotRotation);
    SmartDashboard.putNumber("drive to point/rotation", rotation);
    m_subsystem.arcadeDrive(speed, rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(drivepid.atSetpoint() && turnpid.atSetpoint()){
      return true;
    } else {
      return false;
    }
  }
}
