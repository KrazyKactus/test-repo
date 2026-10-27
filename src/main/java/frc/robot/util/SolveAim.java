// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.ShooterConstants.TurretConstants;
import frc.robot.subsystems.drive.Drive;

/** Add your docs here. */
public class SolveAim {

    public record Aim(double angle, double velocity, double distance) {}

    public static final Aim solveAim(Drive drive, Translation2d target) {
        ChassisSpeeds field = ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation());
        double rotationalVel = field.omegaRadiansPerSecond;

        Rotation2d heading = drive.getRotation().plus(new Rotation2d(rotationalVel * TurretConstants.kDelay));
        Translation2d robotPos = drive.getPose()
                .getTranslation()
                .plus(new Translation2d(
                        field.vxMetersPerSecond * TurretConstants.kDelay,
                        field.vyMetersPerSecond * TurretConstants.kDelay));

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

        double wrapped =
                MathUtil.inputModulus(desiredHeading, TurretConstants.minAngle, TurretConstants.minAngle + 2 * Math.PI);
        double angle = MathUtil.clamp(wrapped, TurretConstants.minAngle, TurretConstants.maxAngle);

        double r2 = Math.max(Math.pow(toTarget.getX(), 2) + Math.pow(toTarget.getY(), 2), 1e-3);
        double bearingRate = (toTarget.getY() * vx - toTarget.getX() * vy) / r2;
        double turretRate = bearingRate - rotationalVel;
        if (angle != wrapped) turretRate = 0.0;

        return new Aim(angle, turretRate, toTarget.getNorm());
    }
}
