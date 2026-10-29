// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants.HoodConstants;
import frc.robot.Constants.ShooterConstants.TurretConstants;
import frc.robot.generated.TunerConstants;
import org.littletonrobotics.junction.Logger;

public class TurretSubsystem extends SubsystemBase {
    /** Creates a new TurretSubsystem. */
    private static final TalonFX turretMotor = new TalonFX(TurretConstants.kTurretID, TunerConstants.kCANBus);

    private static final CANcoder kAbsoluteEncoder =
            new CANcoder(TurretConstants.kThroughboreID, TunerConstants.kCANBus);

    private static final TalonFXConfiguration turretConfig = new TalonFXConfiguration();

    private static final CANcoderConfiguration absoluteEncoderConfig = new CANcoderConfiguration();

    private static PositionDutyCycle positionRequest = new PositionDutyCycle(0);
    private static final NeutralOut neutralRequest = new NeutralOut();

    public static double angle;

    public TurretSubsystem() {
        var currentConfigs = turretConfig.CurrentLimits;
        currentConfigs.SupplyCurrentLimitEnable = true;
        currentConfigs.SupplyCurrentLimit = Constants.kSupplyCurrentLimit;

        var motorOutputConfigs = turretConfig.MotorOutput;
        motorOutputConfigs.Inverted = TurretConstants.kInverted;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

        var feedbackConfigs = turretConfig.Feedback;
        feedbackConfigs.SensorToMechanismRatio = TurretConstants.kGearRatio;

        var limitConfigs = turretConfig.SoftwareLimitSwitch;
        limitConfigs.ForwardSoftLimitEnable = true;
        limitConfigs.ForwardSoftLimitThreshold = TurretConstants.maxAngle / (2 * Math.PI);
        limitConfigs.ReverseSoftLimitEnable = true;
        limitConfigs.ReverseSoftLimitThreshold = TurretConstants.minAngle / (2 * Math.PI);

        var slot0Configs = turretConfig.Slot0;
        slot0Configs.kP = HoodConstants.kP;
        slot0Configs.kI = HoodConstants.kI;
        slot0Configs.kD = HoodConstants.kD;

        absoluteEncoderConfig.MagnetSensor.MagnetOffset = TurretConstants.kMagnetOffset;

        turretMotor.clearStickyFaults();
        turretMotor.getConfigurator().apply(turretConfig);

        turretMotor.setPosition(kAbsoluteEncoder.getPosition().getValueAsDouble() / Math.PI);
    }

    public final void setAngle(double position, double velocity) {
        turretMotor.setControl(positionRequest.withPosition(position).withVelocity(velocity));
    }

    public final void stop() {
        turretMotor.setControl(neutralRequest);
        turretMotor.stopMotor();
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Turret/Turret Angle", turretMotor.getPosition().getValueAsDouble());
        Logger.recordOutput(
                "Turret/Supply Current", turretMotor.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput(
                "Turret/Stator Current", turretMotor.getStatorCurrent().getValueAsDouble());
        // This method will be called once per scheduler run
    }
}
