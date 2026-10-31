// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.IntakeConstants.FieldConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the constants are needed, to
 * reduce verbosity.
 */
// Ball effiency: 1.15
public final class Constants {
    public static final double kGravity = 9.8;
    public static final int kSupplyCurrentLimit = 40;

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static class ShooterConstants {
        public static class FlywheelConstants {
            public static final int kFlywheelID = 22;
            public static final int kFollowerID = 23;

            public static final double kP = 0.3, kI = 0, kD = 0.0001;
            public static final double kS = 1, kV = 0.1;

            public static final InvertedValue kInverted = InvertedValue.CounterClockwise_Positive;

            public static final int kStatorCurrentLimit = 40;

            public static final double kEfficiency = 0.85; // 80% efficiency

            public static final int kIdleVelocity = 20; // rps

            public static final double kTolerance = 1;

            public static final double kBottomWheelR1 = 0.0381; // 2in needs to be 3in
            public static final double kTopWheelR1 = 0.0127; // 1in

            public static final double dy = FieldConstants.hubHeight + FieldConstants.aboveHeight;
            public static final double vy = Math.sqrt(2 * Constants.kGravity * dy);
            public static final double kTime =
                    (vy + Math.sqrt(2 * Constants.kGravity * FieldConstants.aboveHeight)) / (Constants.kGravity);
        }

        public static class HoodConstants {
            public static final int kHoodID = 21;

            public static final InvertedValue kInverted = InvertedValue.CounterClockwise_Positive;

            public static final int kStatorCurrentLimit = 40;

            public static final double kGearRatio = 26;

            public static final double kMaxAngle = 35;
            public static final double kIdlePosition = 0.005;

            public static final double kP = 10, kI = 0, kD = 0;
        }

        public static class TurretConstants {
            public static final int kTurretID = 24;

            public static final int kThroughboreID = 20;

            public static final InvertedValue kInverted = InvertedValue.CounterClockwise_Positive;

            public static final int kStatorCurrentLimit = 40;

            public static final double kGearRatio = 56.25;

            public static final double minAngle = Units.degreesToRadians(-90);
            public static final double maxAngle = Units.degreesToRadians(90);

            public static final double kp = 10, kI = 0, kD = 0;

            public static final double kMagnetOffset = -0.422;

            public static final double kDelay = 0; // input delay on shot
            public static final InterpolatingDoubleTreeMap timeOfFlight = new InterpolatingDoubleTreeMap();

            static {
                timeOfFlight.put(1.0, 1.0);
                timeOfFlight.put(1.5, 1.0);
                timeOfFlight.put(2.0, 1.0);
                timeOfFlight.put(2.5, 1.0);
                timeOfFlight.put(3.0, 1.0);
            }

            public static final Translation2d robotToTurret = new Translation2d(1, 1);
        }
    }

    public static class DyeRotorConstants {
        public static final int kDyeRotorID = 30;

        public static final double kForwardSpeed = 1;
        public static final double kReverseSpeed = -0.3;
        public static final Current kForwardAmperage = Current.ofBaseUnits(180, Amps);
        public static final Current kReverseAmperage = Current.ofBaseUnits(-80, Amps);

        public static final int kStatorCurrentLimit = 180;
        public static final int kSupplyCurrentLimit = 80;

        public static final InvertedValue kInverted = InvertedValue.CounterClockwise_Positive;
    }

    public static class IntakeConstants {
        public static class DeployConstants {
            public static final int kDeployID = 40;

            public static final InvertedValue kInverted = InvertedValue.CounterClockwise_Positive;
            public static final int kStatorCurrentLimit = 160;
            public static final int kSupplyCurrentLimit = 80;

            public static final double kZero = 0.3; // to be tested originally 0.3
            public static final double kExtended = 0;
            public static final double kSpeed = 0.3;

            public static final double kTolerance = 0.03;

            public static final double kP = 16, kI = 0, kD = 0.15;
            public static final double kG = 1.5; // originally 1.65
            public static final GravityTypeValue kGravityType = GravityTypeValue.Arm_Cosine;
            public static final double kOffset = -0.03;

            public static final double kFF = -1.8;

            public static final double kMechRatio = 14;
        }

        public static class RollerConstants {
            public static final int kRollerID = 41;

            public static final InvertedValue kInverted = InvertedValue.Clockwise_Positive;
            public static final int kStatorCurrentLimit = 40;

            public static final double kForwardSpeed = 0.80;
            public static final double kAgitateSpeed = 0.8;
            public static final double kReverseSpeed = -0.35;
        }

        public class FieldConstants {
            public static final double hubHeight = 1.3;
            public static final double aboveHeight = 1;

            public static final Translation2d redHub = new Translation2d(12, 4.03);
            ;
            public static final Translation2d bluehub = new Translation2d(4.62, 4.03);
        }
    }
}
