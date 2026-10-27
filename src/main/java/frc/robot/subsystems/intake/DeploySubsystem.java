package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants.DeployConstants;
import frc.robot.generated.TunerConstants;

public class DeploySubsystem extends SubsystemBase {

    private static final TalonFX deployMotor = new TalonFX(DeployConstants.kDeployID, TunerConstants.kCANBus);
    private static TalonFXConfiguration deployConfigs = new TalonFXConfiguration();

    private static PositionVoltage positionRequest = new PositionVoltage(0);
    private static DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
    private static NeutralOut neutralRequest = new NeutralOut();

    public DeploySubsystem() {
        // Configure voltage, current, brake mode and follow motor
        var motorOutputConfigs = deployConfigs.MotorOutput;
        motorOutputConfigs.Inverted = DeployConstants.kInverted;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

        var currentLimitConfigs = deployConfigs.CurrentLimits;
        currentLimitConfigs.SupplyCurrentLimitEnable = true;
        currentLimitConfigs.SupplyCurrentLimit = DeployConstants.kSupplyCurrentLimit;
        currentLimitConfigs.StatorCurrentLimitEnable = true;
        currentLimitConfigs.StatorCurrentLimit = DeployConstants.kStatorCurrentLimit;

        deployConfigs.Feedback.SensorToMechanismRatio = DeployConstants.kMechRatio;

        var slot0Configs = deployConfigs.Slot0;
        slot0Configs.kP = DeployConstants.kP;
        slot0Configs.kI = DeployConstants.kI;
        slot0Configs.kD = DeployConstants.kD;

        slot0Configs.GravityType = DeployConstants.kGravityType;
        slot0Configs.kG = DeployConstants.kG;
        slot0Configs.GravityArmPositionOffset = DeployConstants.kOffset;

        deployMotor.clearStickyFaults();
        deployMotor.getConfigurator().apply(deployConfigs);

        // deployMotor.setPosition(0);
        deployMotor.setPosition(DeployConstants.kZero);
    }

    public double getError() {
        return deployMotor.getClosedLoopError().getValue();
    }

    public void setPosition(double position) {
        deployMotor.setControl(positionRequest.withPosition(position));
    }

    public void setSpeed(double output) {
        deployMotor.setControl(dutyCycleRequest.withOutput(output));
    }

    public void stop() {
        deployMotor.setControl(neutralRequest);
        deployMotor.stopMotor();
    }

    public boolean isExtended() {
        return Math.abs(deployMotor.getPosition().getValueAsDouble() - DeployConstants.kExtended)
                <= DeployConstants.kTolerance;
    }

    public boolean isRetracted() {
        return Math.abs(deployMotor.getPosition().getValueAsDouble() - DeployConstants.kZero)
                <= DeployConstants.kTolerance;
    }

    public boolean isStalled() {
        return Math.abs(deployMotor.getTorqueCurrent().getValueAsDouble()) >= 60;
    }

    public boolean isClear() {
        return deployMotor.getPosition().getValueAsDouble() < 0.174;
    }

    @Override
    public void periodic() {}
}
