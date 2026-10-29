// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants.FlywheelConstants;
import frc.robot.generated.TunerConstants;
import org.littletonrobotics.junction.Logger;

public class FlywheelSubsystem extends SubsystemBase {
    /** CreatesFlywheelSubsystem extends SubsystemBase { /** Creates a new ShooterSubsystem. */
    private static final TalonFX flywheelMotor = new TalonFX(FlywheelConstants.kFlywheelID, TunerConstants.kCANBus);

    private static final TalonFX followerMotor = new TalonFX(FlywheelConstants.kFollowerID, TunerConstants.kCANBus);
    private static final TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    private static TalonFXConfiguration followerConfig;

    private static VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private static NeutralOut neutralRequest = new NeutralOut();
    private static Follower followRequest = new Follower(FlywheelConstants.kFlywheelID, MotorAlignmentValue.Opposed);

    public double vx;
    private double velocity;
    private double rps;

    public FlywheelSubsystem() {
        var currentConfigs = flywheelConfig.CurrentLimits;
        currentConfigs.SupplyCurrentLimitEnable = true;
        currentConfigs.SupplyCurrentLimit = Constants.kSupplyCurrentLimit;

        var motorOutputConfigs = flywheelConfig.MotorOutput;
        motorOutputConfigs.Inverted = FlywheelConstants.kInverted;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;

        var slot0Configs = flywheelConfig.Slot0;
        slot0Configs.kP = FlywheelConstants.kP;
        slot0Configs.kI = FlywheelConstants.kI;
        slot0Configs.kD = FlywheelConstants.kD;
        slot0Configs.kS = FlywheelConstants.kS;
        slot0Configs.kV = FlywheelConstants.kV;

        followerConfig = flywheelConfig.clone();

        flywheelMotor.clearStickyFaults();
        followerMotor.clearStickyFaults();
        flywheelMotor.getConfigurator().apply(flywheelConfig);
        followerMotor.getConfigurator().apply(followerConfig);
    }

    public double getRPS() {
        return this.rps;
    }

    public double calculateXVelocity(double dx) {
        this.vx = dx / FlywheelConstants.kTime;
        return this.vx;
    }

    public double calculateVelocity(double vy, double vx) { // Pythagorean theorem to calculate the velocity of the ball
        this.velocity = Math.sqrt(Math.pow(vx, 2) + Math.pow(vy, 2));
        return this.velocity;
    }

    public double calculateRPS(double v) {
        double angularWheelSpeed = (v)
                / (FlywheelConstants.kEfficiency
                        * (((FlywheelConstants.kBottomWheelR1 + FlywheelConstants.kTopWheelR1) / 2)));
        this.rps = angularWheelSpeed / (2 * Math.PI);
        return this.rps;
    }

    public double getXVelocity() {
        return this.vx;
    }

    public void setVelocity(double rps) {
        flywheelMotor.setControl(velocityRequest.withVelocity(rps));
        followerMotor.setControl(followRequest);
    }

    public void stop() {
        flywheelMotor.setControl(neutralRequest);
        flywheelMotor.setControl(neutralRequest);
        flywheelMotor.stopMotor();
        followerMotor.stopMotor();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        Logger.recordOutput("Flywheel/Calculated/X Velocity", this.vx);
        Logger.recordOutput("Flywheel/Calculated/Y Velocity", FlywheelConstants.vy);
        Logger.recordOutput("Flywheel/Calculated/Velocity", this.velocity);
        Logger.recordOutput("Flywheel/Calculated/RPS", this.rps);
        Logger.recordOutput("Flywheel/Velocity", flywheelMotor.getVelocity().getValueAsDouble());
        Logger.recordOutput(
                "Flywheel/Supply Current", flywheelMotor.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput(
                "Flywheel/Stator Current", flywheelMotor.getStatorCurrent().getValueAsDouble());
    }
}
