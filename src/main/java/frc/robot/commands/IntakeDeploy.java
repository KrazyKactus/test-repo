// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.DeploySubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeDeploy extends Command {
    /** Creates a new IntakeDeploy. */
    private final DeploySubsystem sys_deploy;

    private final double setpoint;

    public IntakeDeploy(DeploySubsystem deploy, double setpoint) {
        this.sys_deploy = deploy;
        this.setpoint = setpoint;
        addRequirements(this.sys_deploy);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.sys_deploy.setPosition(this.setpoint);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted) this.sys_deploy.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.sys_deploy.isStalled() && this.sys_deploy.isExtended();
    }
}
