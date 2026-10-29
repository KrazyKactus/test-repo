// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants.RollerConstants;
import frc.robot.generated.TunerConstants;
import org.littletonrobotics.junction.Logger;

public class RollerSubsystem extends SubsystemBase {
    /** Creates a new RollerSubsystem. */
    private final TalonFX rollerMotor = new TalonFX(RollerConstants.kRollerID, TunerConstants.kCANBus);

    private final TalonFXConfiguration rollerConfig = new TalonFXConfiguration();

    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
    private final NeutralOut neutralRequest = new NeutralOut();

    public RollerSubsystem() {
        var motorOutputConfigs = rollerConfig.MotorOutput;
        motorOutputConfigs.Inverted = RollerConstants.kInverted;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        motorOutputConfigs.PeakForwardDutyCycle = RollerConstants.kForwardSpeed;
        motorOutputConfigs.PeakReverseDutyCycle = RollerConstants.kReverseSpeed;

        var currentLimitConfigs = rollerConfig.CurrentLimits;
        currentLimitConfigs.SupplyCurrentLimitEnable = true;
        currentLimitConfigs.SupplyCurrentLimit = Constants.kSupplyCurrentLimit;

        rollerMotor.clearStickyFaults();
        rollerMotor.getConfigurator().apply(rollerConfig);
    }

    public double getError() {
        return RollerConstants.kForwardSpeed - rollerMotor.getDutyCycle().getValue();
    }

    public void setSpeed(double speed) {
        rollerMotor.setControl(dutyCycleRequest.withOutput(speed));
    }

    public boolean isRolling() {
        return Math.abs(rollerMotor.get()) > 0;
    }

    public void stop() {
        rollerMotor.setControl(neutralRequest);
        rollerMotor.stopMotor();
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Roller/Speed", rollerMotor.getDutyCycle().getValueAsDouble());
        Logger.recordOutput("Roller/Rolling Status", this.isRolling());
        Logger.recordOutput(
                "Roller/Supply Current", rollerMotor.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput(
                "Roller/Stator Current", rollerMotor.getStatorCurrent().getValueAsDouble());
    }
}
