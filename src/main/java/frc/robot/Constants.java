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
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotBase;

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

            public static final double kEfficiency = 0.8; // 80% efficiency

            public static final int kIdleVelocity = 20; // rps

            public static final double kTolerance = 1;

            // moments of inertia calc
            public static final double kBottomWheelR1 = 0.0508; // 2in
            public static final double kBottomWheelR2 = 0.0508 - 0.0127; // 2in - 0.5in (width)
            public static final double kBottomWheelMass = 0.096;

            public static final double kTopWheelR1 = 0.0254; // 1in
            public static double kTopWheelR2; // wheels are small and thus can be approximated as solid
            public static final double kTopWheelMass = kBottomWheelMass * 0.3; // approx 30% of the bottom wheel mass

            public static final double kStainlessFlywheelInertia = 0.0007901271; // kg*m^2
            public static final double kCutInertialFlywheelMass = 0.612; // Idk
            public static final double kCutInertialFlywheelRadius = 0.0762; // 3in

            public static final double kTopWheelInertia = (kTopWheelMass * (Math.pow(kTopWheelR1, 2))) / 2; // 1/2mr^2
            public static final double kBottomWheelInertia =
                    (kBottomWheelMass * (Math.pow(kBottomWheelR1, 2))) / 2; // 1/2M(r1^2+r2^2)
            public static final double kCutInertialFlywheelInertia =
                    (kCutInertialFlywheelMass * (Math.pow(kCutInertialFlywheelRadius, 2))) / 2;

            public static final double kTotalInertia =
                    kTopWheelInertia * 6 + kBottomWheelInertia * 6 + kCutInertialFlywheelInertia * 2; // 0.0043525kg*m^2
        }

        public static class HoodConstants {
            public static final int kHoodID = 21;

            public static final InvertedValue kInverted = InvertedValue.Clockwise_Positive;

            public static final int kStatorCurrentLimit = 40;

            public static final double kGearRatio = 62.5;

            public static final double kIdlePosition = 0.005;

            public static final double kP = 70, kI = 0, kD = 0.18;
        }

        public static class TurretConstants {
            public static final int kTurretID = 24;

            public static final int kThroughboreID = 20;

            public static final InvertedValue kInverted = null;

            public static final int kStatorCurrentLimit = 40;

            public static final double kGearRatio = 0;

            public static final double minAngle = Units.degreesToRadians(-270);
            public static final double maxAngle = Units.degreesToRadians(270);

            public static final double kp = 0, kI = 0, kD = 0;

            public static final double kMagnetOffset = 0;

            public static final double kDelay = 0; // input delay on shot
            public static final InterpolatingDoubleTreeMap timeOfFlight = new InterpolatingDoubleTreeMap();

            static {
                timeOfFlight.put(1.0, 1.0);
                timeOfFlight.put(2.0, 3.0);
            }

            public static final Translation2d robotToTurret = null;
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

            public static double kZero = 0.3; // to be tested originally 0.3
            public static double kExtended = 0;
            public static double kSpeed = 0.3;

            public static double kTolerance = 0.03;

            public static double kP = 16, kI = 0, kD = 0.15;
            public static double kG = 1.65; // originally 2
            public static GravityTypeValue kGravityType = GravityTypeValue.Arm_Cosine;
            public static double kOffset = -0.03;

            public static double kFF = -1.8;

            public static double kMechRatio = 14;
        }

        public static class RollerConstants {
            public static final int kRollerID = 41;

            public static final InvertedValue kInverted = InvertedValue.Clockwise_Positive;
            public static final int kStatorCurrentLimit = 40;

            public static final double kForwardSpeed = 0.95;
            public static final double kAgitateSpeed = 0.8;
            public static final double kReverseSpeed = -0.35;
        }

        public static class VisionConstants {

            public static final String leftCamera = "Camera1";
            public static final String rightCamera = "Camera4";

            public static edu.wpi.first.math.Vector<N3> kSingleTagStdDevs = VecBuilder.fill(4, 4, 4);
            public static edu.wpi.first.math.Vector<N3> kMultiTagStdDevs = VecBuilder.fill(4, 4, 4);

            public static final Transform3d kLeftTranslation = new Transform3d(
                    new Translation3d(
                            Units.inchesToMeters(10.367), // X: centered front-back
                            Units.inchesToMeters(7.261), // Y: 7 inches to the right
                            Units.inchesToMeters(20.164) // Z: height above ground
                            ),
                    new Rotation3d(
                            0, // Roll
                            Units.degreesToRadians(30), // Pitch (tilt up)
                            Units.degreesToRadians(22) // Yaw (right)
                            ));
            public static final Transform3d kRightTranslation = new Transform3d(
                    new Translation3d(
                            Units.inchesToMeters(10.367), // X: centered front-back
                            Units.inchesToMeters(-7.261), // Y: 7 inches to the left (negative)
                            Units.inchesToMeters(20.164) // Z: height above ground
                            ),
                    new Rotation3d(
                            0, // Roll
                            Units.degreesToRadians(30), // Pitch (tilt down)
                            Units.degreesToRadians(-22) // Yaw (facing left)
                            ));
        }

        public class FieldConstants {
            public static final Translation2d redHub = new Translation2d();
            public static final Translation2d bluehub = new Translation2d();

            public static final Translation2d redWall = new Translation2d();
            public static final Translation2d blueWall = new Translation2d();
        }
    }
}
