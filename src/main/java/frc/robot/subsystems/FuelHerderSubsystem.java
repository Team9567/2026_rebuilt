package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.FuelHerderConstants;

public class FuelHerderSubsystem extends SubsystemBase {
    SparkMax leftherderMotor;
    SparkMax rightherderMotor;
    ProfiledPIDController leftpid = new ProfiledPIDController(0.25, 0, 0, new Constraints(25, 25));
    ProfiledPIDController rightpid = new ProfiledPIDController(0.25, 0, 0, new Constraints(25, 25));
    boolean isHomed = false;

    public FuelHerderSubsystem() {
        if (FuelHerderConstants.kIsEnabled) {
            leftherderMotor = new SparkMax(FuelHerderConstants.kLeftHerderMotor, MotorType.kBrushless);
            rightherderMotor = new SparkMax(FuelHerderConstants.kRightHerderMotor, MotorType.kBrushless);
            SparkMaxConfig config = new SparkMaxConfig();
            config
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);

            config.openLoopRampRate(FuelHerderConstants.kMotorRampTime);
            config.inverted(false);
            config.softLimit
                    .forwardSoftLimit(FuelHerderConstants.kLeftForwardSoftLimit)
                    .reverseSoftLimit(FuelHerderConstants.kReverseSoftLimit)
                    .forwardSoftLimitEnabled(false)
                    .reverseSoftLimitEnabled(false);
            leftherderMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            config.inverted(true);
            config.softLimit
                    .forwardSoftLimit(FuelHerderConstants.kRightForwardSoftLimit)
                    .reverseSoftLimit(FuelHerderConstants.kReverseSoftLimit)
                    .forwardSoftLimitEnabled(false)
                    .reverseSoftLimitEnabled(false);
            rightherderMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            setDefaultCommand(homeArms());
        }

    }

    public Command homeArms() {
        if (FuelHerderConstants.kIsEnabled) {
            return run(() -> {
                setLeft(FuelHerderConstants.kHomeArmSpeed);
                setRight(FuelHerderConstants.kHomeArmSpeed);
            });
        } else {
            return Commands.none();
        }

    }

    public Command extendArms() {
        return armPid(FuelHerderConstants.kLeftForwardSoftLimit, FuelHerderConstants.kRightForwardSoftLimit);
    }

    public Command retractArms() {
        return armPid(0, 0);
    }

    public Command armPid(double leftTarget, double rightTarget) {
        if (FuelHerderConstants.kIsEnabled) {
            return startRun(() -> {
                leftpid.reset(leftherderMotor.getEncoder().getPosition());
                rightpid.reset(rightherderMotor.getEncoder().getPosition());

            }, () -> {
                double leftPos = leftherderMotor.getEncoder().getPosition();
                double rightPos = rightherderMotor.getEncoder().getPosition();
                double leftSpeed = leftpid.calculate(leftPos, leftTarget);
                double rightSpeed = rightpid.calculate(rightPos, rightTarget);

                SmartDashboard.putNumber("FuelHerder/left speed", leftSpeed);
                SmartDashboard.putNumber("FuelHerder/right speed", rightSpeed);
                SmartDashboard.putNumber("FuelHerder/left position", leftPos);
                SmartDashboard.putNumber("FuelHerder/right position", rightPos);
                SmartDashboard.putNumber("FuelHerder/left target", leftTarget);
                SmartDashboard.putNumber("FuelHerder/right target", rightTarget);

                rightSpeed = MathUtil.clamp(rightSpeed, -0.50, 0.50);
                leftSpeed = MathUtil.clamp(leftSpeed, -0.50, 0.50);
                setLeft(leftSpeed);
                setRight(rightSpeed);
            });
        } else {
            return Commands.none();
        }
    }

    public void setLeft(double speed) {
        if (FuelHerderConstants.kIsEnabled) {
            leftherderMotor.set(speed);
        }
    }

    public void setRight(double speed) {
        if (FuelHerderConstants.kIsEnabled) {
            rightherderMotor.set(speed);
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (FuelHerderConstants.kIsEnabled) {
            SmartDashboard.putBoolean("FuelHerder/homed", isHomed);
            SmartDashboard.putNumber("FuelHerder/rightdistance", rightherderMotor.getEncoder().getPosition());
            SmartDashboard.putNumber("FuelHerder/leftdistance", leftherderMotor.getEncoder().getPosition());
            SmartDashboard.putNumber("FuelHerder/rightcurrent", rightherderMotor.getOutputCurrent());
            SmartDashboard.putNumber("FuelHerder/leftcurrent", leftherderMotor.getOutputCurrent());
            if (!isHomed) {
                // Homing
                if (rightherderMotor.getOutputCurrent() > FuelHerderConstants.kRightCurrentThreshold
                        && leftherderMotor.getOutputCurrent() > FuelHerderConstants.kLeftCurrentThreshold) {
                    // Motors have reached hard stop, reset encoders, enable soft limigts, and mark
                    // homing complete
                    rightherderMotor.getEncoder().setPosition(0);
                    leftherderMotor.getEncoder().setPosition(0);
                    SparkMaxConfig config = new SparkMaxConfig();
                    config.softLimit
                            .forwardSoftLimitEnabled(true)
                            .reverseSoftLimitEnabled(true);
                    leftherderMotor.configure(config, ResetMode.kNoResetSafeParameters,
                            PersistMode.kNoPersistParameters);
                    rightherderMotor.configure(config, ResetMode.kNoResetSafeParameters,
                            PersistMode.kNoPersistParameters);
                    var homingCommand = getCurrentCommand();
                    setDefaultCommand(retractArms());
                    CommandScheduler.getInstance().cancel(homingCommand);

                    isHomed = true;
                }
            }
        }
    }
}
