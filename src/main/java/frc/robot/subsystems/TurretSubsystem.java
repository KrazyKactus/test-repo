// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants.HoodConstants;
import frc.robot.Constants.ShooterConstants.TurretConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;

public class TurretSubsystem extends SubsystemBase {
  /** Creates a new TurretSubsystem. */

  private static final TalonFX kTurretMotor = new TalonFX(TurretConstants.kTurretID, TunerConstants.kCANBus);

  private static final CANcoder kAbsoluteEncoder = new CANcoder(TurretConstants.kThroughboreID, TunerConstants.kCANBus);

  private static final TalonFXConfiguration turretConfig = new TalonFXConfiguration();

  private static final CANcoderConfiguration absoluteEncoderConfig = new CANcoderConfiguration();

  private static PositionDutyCycle positionRequest = new PositionDutyCycle(0);

  public static double angle;

  public record Aim(double angle, double velocity, double distance) {
  }

  public TurretSubsystem() {
    var currentConfigs = turretConfig.CurrentLimits;
    currentConfigs.SupplyCurrentLimitEnable = true;
    currentConfigs.SupplyCurrentLimit = Constants.kSupplyCurrentLimit;

    var motorOutputConfigs = turretConfig.MotorOutput;
    motorOutputConfigs.Inverted = TurretConstants.kInverted;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    var feedbackConfigs = turretConfig.Feedback;
    feedbackConfigs.SensorToMechanismRatio = TurretConstants.kGearRatio;

    var slot0Configs = turretConfig.Slot0;
    slot0Configs.kP = HoodConstants.kP;
    slot0Configs.kI = HoodConstants.kI;
    slot0Configs.kD = HoodConstants.kD;

    absoluteEncoderConfig.MagnetSensor.MagnetOffset = TurretConstants.kMagnetOffset;

    kTurretMotor.clearStickyFaults();
    kTurretMotor.getConfigurator().apply(turretConfig);
  }

  public static Aim solveAim(Drive drive, Translation2d target) {
    ChassisSpeeds field = ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation());
    double rotationalVel = field.omegaRadiansPerSecond;

    Rotation2d heading = drive.getRotation().plus(new Rotation2d(rotationalVel * TurretConstants.kDelay));
    Translation2d robotPos = drive.getPose().getTranslation().plus(new Translation2d(
        field.vxMetersPerSecond * TurretConstants.kDelay, field.vyMetersPerSecond * TurretConstants.kDelay));

    Translation2d offset = TurretConstants.robotToTurret.rotateBy(heading);
    Translation2d turretPos = robotPos.plus(offset);

    double vx = field.vxMetersPerSecond + rotationalVel * offset.getY();
    double vy = field.vyMetersPerSecond + rotationalVel * offset.getX();
    Translation2d turretVel = new Translation2d(vx, vy);

    Translation2d virtualTarget = target;
    for (int i = 0; i < 4; i++) {
      double distance = virtualTarget.minus(turretPos).getNorm();
      double tof = TurretConstants.timeOfFlight.get(distance);
      virtualTarget = target.minus(turretVel.times(tof));
    }

    Translation2d toTarget = virtualTarget.minus(turretPos);

    double desiredHeading = toTarget.getAngle().minus(heading).getRadians();

    double wrapped = MathUtil.inputModulus(desiredHeading, TurretConstants.minAngle,
        TurretConstants.minAngle + 2 * Math.PI);
    double angle = MathUtil.clamp(wrapped, TurretConstants.minAngle, TurretConstants.maxAngle);

    double r2 = Math.max(Math.pow(toTarget.getX(), 2) + Math.pow(toTarget.getY(), 2), 1e-3);
    double bearingRate = (toTarget.getY() * vx - toTarget.getX() * vy) / r2;
    double turretRate = bearingRate - rotationalVel;
    if (angle != wrapped)
      turretRate = 0.0;

    return new Aim(angle, turretRate, toTarget.getNorm());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
