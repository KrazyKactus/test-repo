// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
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

    private static final double heightOfHub = 1.3; // Big H in metres
    private static final double heightAboveHub = 1; // Little h in metres
    private static final double yDisplacement =
            heightOfHub + heightAboveHub; // The height of the vertex above the ground

    public static double yVelocity =
            Math.sqrt(2 * Constants.kGravity * yDisplacement); // initial velocity calculated through finding at vertex
    public static double xVelocity;
    private static double velocity;
    private static double rpsFromVelocity;

    private static final double time =
            ((yVelocity + Math.sqrt(2 * Constants.kGravity * heightAboveHub)) / Constants.kGravity);

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

    public double getYVelocity() {
        return yVelocity;
    }

    public double getXVelocity() {
        return xVelocity;
    }

    public double getRPS() {
        return rpsFromVelocity;
    }

    public double getError() {
        double error = Math.abs(flywheelMotor.getClosedLoopError().getValue());
        Logger.recordOutput("Flywheel error", error);
        return error;
    }

    public void calculateXVelocity(double xDisplacement) {
        xVelocity = xDisplacement / time;
        Logger.recordOutput("Flywheel X Velocity", xVelocity, MetersPerSecond);
    }

    public void calculateVelocity() { // Pythagorean theorem to calculate the velocity of the ball (speed for thing)
        velocity = Math.sqrt(Math.pow(xVelocity, 2) + Math.pow(yVelocity, 2));
        Logger.recordOutput("Flywheel Velocity", velocity, MetersPerSecond);
    }

    public void calculateRPS() {
        double angularWheelSpeed =
                (velocity) / (FlywheelConstants.kEfficiency * (((FlywheelConstants.kBottomWheelR1 + FlywheelConstants.kTopWheelR1) / 2)));
        rpsFromVelocity = angularWheelSpeed / (2*Math.PI);
        Logger.recordOutput("Flywheel RPS", rpsFromVelocity, RotationsPerSecond);
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
        Logger.recordOutput("Flywheel Speed", flywheelMotor.getVelocity().getValue());
    }
}
