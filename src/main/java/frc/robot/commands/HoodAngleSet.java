// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.HoodSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HoodAngleSet extends Command {
    /** Creates a new HoodAngleSet. */
    private HoodSubsystem sys_hood;

    private double setpoint;

    public HoodAngleSet(HoodSubsystem hood, double setpoint) {
        this.sys_hood = hood;
        this.setpoint = setpoint;
        addRequirements(this.sys_hood);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        this.sys_hood.setAngle(setpoint);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted) new HoodIdle(this.sys_hood);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // is hood at setpoint? return true if yes
        // return this.sys_hood.getError() <= 0.001;
        return false;
    }
}
