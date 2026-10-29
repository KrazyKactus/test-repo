// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ShooterConstants.FlywheelConstants;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import java.util.function.DoubleSupplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HoodAimShoot extends ParallelCommandGroup {
    /** Creates a new HoodAimShoot. */
    private final HoodSubsystem sys_hood;

    private final FlywheelSubsystem sys_flywheel;
    private final double dx;
    private final double vx;
    private final double rps;

    private final FlywheelSpin flywheelCommand;
    private final HoodAngleSet hoodCommand;

    public HoodAimShoot(HoodSubsystem hood, FlywheelSubsystem flywheel, DoubleSupplier distance) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        this.sys_hood = hood;
        this.sys_flywheel = flywheel;
        this.dx = distance.getAsDouble();
        this.vx = this.sys_flywheel.calculateXVelocity(this.dx);
        this.rps = this.sys_flywheel.calculateRPS(this.sys_flywheel.calculateVelocity(FlywheelConstants.vy, vx));

        this.flywheelCommand = new FlywheelSpin(this.sys_flywheel, this.rps);

        this.hoodCommand = new HoodAngleSet(this.sys_hood, this.sys_hood.calculateAngle(FlywheelConstants.vy, vx));

        addCommands(flywheelCommand, hoodCommand);
    }
}
