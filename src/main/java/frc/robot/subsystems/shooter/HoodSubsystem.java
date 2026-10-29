// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants.HoodConstants;
import frc.robot.generated.TunerConstants;
import org.littletonrobotics.junction.Logger;

public class HoodSubsystem extends SubsystemBase {
    /** Creates a new HoodSubsystem. */
    private static final TalonFX hoodMotor = new TalonFX(HoodConstants.kHoodID, TunerConstants.kCANBus);

    private static final TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

    private static PositionDutyCycle positionRequest = new PositionDutyCycle(0);

    public double angle;

    public HoodSubsystem() {
        var currentConfigs = hoodConfig.CurrentLimits;
        currentConfigs.SupplyCurrentLimitEnable = true;
        currentConfigs.SupplyCurrentLimit = Constants.kSupplyCurrentLimit;

        var motorOutputConfigs = hoodConfig.MotorOutput;
        motorOutputConfigs.Inverted = HoodConstants.kInverted;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

        var feedbackConfigs = hoodConfig.Feedback;
        feedbackConfigs.SensorToMechanismRatio = HoodConstants.kGearRatio;

        var slot0Configs = hoodConfig.Slot0;
        slot0Configs.kP = HoodConstants.kP;
        slot0Configs.kI = HoodConstants.kI;
        slot0Configs.kD = HoodConstants.kD;

        hoodMotor.clearStickyFaults();
        hoodMotor.getConfigurator().apply(hoodConfig);
        hoodMotor.setPosition(0);
    }

    public double getError() {
        return hoodMotor.getClosedLoopError().getValue();
    }

    public double getAngle() {
        return this.angle;
    }

    public double calculateAngle(
            double y, double x) { // Calculates the angle of the ball and relates it to the angle of the hood
        this.angle = (Math.PI - ((Math.PI / 2) + (Math.atan2(y, x)))) / (2 * Math.PI);
        return this.angle;
    }

    public void setAngle(double target) {
        hoodMotor.setControl(positionRequest.withPosition(target));
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Hood/Calculated/Hood Angle", this.angle);
        Logger.recordOutput("Hood/Angle", hoodMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("Hood/Supply Current", hoodMotor.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("Hood/Stator Current", hoodMotor.getStatorCurrent().getValueAsDouble());
        // This method will be called once per scheduler run
    }
}
