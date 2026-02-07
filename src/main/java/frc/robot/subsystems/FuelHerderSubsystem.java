package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.FuelHerderConstants;

public class FuelHerderSubsystem {
    SparkMax leftherderMotor = new SparkMax(FuelHerderConstants.kLeftHerderMotor, MotorType.kBrushless);
    SparkMax rightherderMotor = new SparkMax(FuelHerderConstants.kRightHerderMotor, MotorType.kBrushless);

    public FuelHerderSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(20);
        config.softLimit
                .forwardSoftLimit(FuelHerderConstants.kForwardSoftLimit)
                .reverseSoftLimit(FuelHerderConstants.kReverseSoftLimit)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimitEnabled(true);
        config.openLoopRampRate(FuelHerderConstants.kMotorRampTime);
        config.encoder.positionConversionFactor(FuelHerderConstants.kPositionConversionFactor);
        config.inverted(false);
        leftherderMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        config.inverted(true);
        rightherderMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setLeft(double speed) {
        leftherderMotor.set(speed);

    }

    public void setRight(double speed) {
        rightherderMotor.set(speed);
    }
}
