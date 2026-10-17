// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DyeRotorConstants;
import frc.robot.generated.TunerConstants;

public class DyeRotorSubsystem extends SubsystemBase {
    /** Creates a new DyeRotorSubsystem. */
    private final TalonFX dyeRotorMotor = new TalonFX(DyeRotorConstants.kDyeRotorID, TunerConstants.kCANBus);

    TalonFXConfiguration dyeRotorConfig = new TalonFXConfiguration();

    public final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0).withEnableFOC(true);
    public final NeutralOut neutralRequest = new NeutralOut();
    public final TorqueCurrentFOC torqueRequest = new TorqueCurrentFOC(0);

    public DyeRotorSubsystem() {

        var motorOutputConfigs = dyeRotorConfig.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        motorOutputConfigs.PeakForwardDutyCycle = DyeRotorConstants.kForwardSpeed;
        motorOutputConfigs.PeakReverseDutyCycle = DyeRotorConstants.kReverseSpeed;
        motorOutputConfigs.Inverted = DyeRotorConstants.kInverted;

        var currentLimitConfigs = dyeRotorConfig.CurrentLimits;
        currentLimitConfigs.StatorCurrentLimitEnable = true;
        currentLimitConfigs.StatorCurrentLimit = DyeRotorConstants.kStatorCurrentLimit;
        currentLimitConfigs.SupplyCurrentLimitEnable = true;
        currentLimitConfigs.SupplyCurrentLimit = DyeRotorConstants.kSupplyCurrentLimit;
        currentLimitConfigs.SupplyCurrentLowerLimit = 40;
        currentLimitConfigs.SupplyCurrentLowerTime = 2;

        dyeRotorMotor.clearStickyFaults();
        dyeRotorMotor.getConfigurator().apply(dyeRotorConfig);
    }

    public double getError() {
        return DyeRotorConstants.kForwardSpeed - dyeRotorMotor.getDutyCycle().getValue();
    }

    public void setSpeed(double speed) {
        dyeRotorMotor.setControl(dutyCycleRequest.withOutput(speed));
    }

    public void setAmps(Current amperes) {
        dyeRotorMotor.setControl(torqueRequest.withOutput(amperes));
    }

    public void stop() {
        dyeRotorMotor.setControl(neutralRequest);
        dyeRotorMotor.stopMotor();
    }
}
