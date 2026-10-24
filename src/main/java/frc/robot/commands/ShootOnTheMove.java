// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.TurretSubsystem.Aim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootOnTheMove extends ParallelCommandGroup {
    /** Creates a new ShootOnTheMove. */
    private final Drive sys_drive;

    private final TurretSubsystem sys_turret;
    private final FlywheelSubsystem sys_flywheel;
    private final HoodSubsystem sys_hood;
    private final Translation2d target;

    private Aim lastAim;

    public ShootOnTheMove(
            Drive drive, TurretSubsystem turret, FlywheelSubsystem flywheel, HoodSubsystem hood, Translation2d target) {
        this.sys_drive = drive;
        this.sys_turret = turret;
        this.sys_flywheel = flywheel;
        this.sys_hood = hood;
        this.target = target;

        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                Commands.run(() -> lastAim = sys_turret.solveAim(this.sys_drive, target)),
                new TurretAngleSet(sys_turret, () -> lastAim.angle(), () -> lastAim.velocity()),
                new HoodAimShoot(sys_hood, sys_flywheel, () -> lastAim.distance()));
    }
}
