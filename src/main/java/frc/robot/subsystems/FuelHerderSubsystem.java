package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.FuelHerderConstants;

public class FuelHerderSubsystem extends SubsystemBase {
    SparkMax leftherderMotor = new SparkMax(FuelHerderConstants.kLeftHerderMotor, MotorType.kBrushless);
    SparkMax rightherderMotor = new SparkMax(FuelHerderConstants.kRightHerderMotor, MotorType.kBrushless);
    boolean isHomed = false;

    public FuelHerderSubsystem() {
        if (FuelHerderConstants.kIsEnabled) {
            SparkMaxConfig config = new SparkMaxConfig();
            config
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);

            config.openLoopRampRate(FuelHerderConstants.kMotorRampTime);
            config.encoder.positionConversionFactor(FuelHerderConstants.kPositionConversionFactor);
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

            setDefaultCommand(retractArms());
        }

    }

    public Command retractArms() {
        if (FuelHerderConstants.kIsEnabled) {
            return run(() -> {
                setLeft(FuelHerderConstants.kRetractArmSpeed);
                setRight(FuelHerderConstants.kRetractArmSpeed);
            });
        } else {
            return Commands.none();
        }

    }

    public Command extendArms() {
        if (FuelHerderConstants.kIsEnabled) {
            return run(() -> {
                setLeft(FuelHerderConstants.kExtendArmSpeed);
                setRight(FuelHerderConstants.kExtendArmSpeed);
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
                    isHomed = true;
                }
            }
        }
    }
}
