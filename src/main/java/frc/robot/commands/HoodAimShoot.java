// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
    private final DoubleSupplier deltaX;

    private final FlywheelSpin flywheelCommand;
    private final HoodAngleSet hoodCommand;

    public HoodAimShoot(HoodSubsystem hood, FlywheelSubsystem flywheel, DoubleSupplier deltaXSupplier) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        this.sys_hood = hood;
        this.sys_flywheel = flywheel;
        this.deltaX = deltaXSupplier;

        this.flywheelCommand = new FlywheelSpin(this.sys_flywheel, this.deltaX);

        // this.angle = this.sys_hood.calculateAngle(this.sys_flywheel.getYVelocity(),
        // this.sys_flywheel.getXVelocity());
        // this.hoodCommand = new HoodAngleSet(this.sys_hood, this.angle);

        this.hoodCommand =
                new HoodAngleSet(this.sys_hood, this.sys_flywheel::getYVelocity, this.sys_flywheel::getXVelocity);

        addCommands(flywheelCommand, hoodCommand);
    }
}
