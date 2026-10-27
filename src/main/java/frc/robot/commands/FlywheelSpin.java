// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.FlywheelSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FlywheelSpin extends Command {
    /** Creates a new FlywheelSpin. */
    private final FlywheelSubsystem sys_flywheel;

    private final double dx;

    public FlywheelSpin(FlywheelSubsystem flywheel, double distance) {
        this.sys_flywheel = flywheel;
        this.dx = distance;
        addRequirements(sys_flywheel);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.sys_flywheel.calculateXVelocity(this.dx);
        this.sys_flywheel.calculateVelocity();
        this.sys_flywheel.calculateRPS();
        this.sys_flywheel.setVelocity(this.sys_flywheel.getRPS());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // when interrupted set flywheel to idle
        if (interrupted)
            this.sys_flywheel.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // is flywheel at desired velocity? return true if yes
        return false;
    }
}
