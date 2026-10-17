// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DyeRotorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DyeRotorSpin extends Command {
    /** Creates a new DyeRotorSpin. */
    private final DyeRotorSubsystem sys_dyeRotor;

    private double setpoint;
    private Current amps;

    private boolean isAmps = false;
    private boolean isSetpoint = false;

    public DyeRotorSpin(DyeRotorSubsystem dyerotor, double setpoint) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.sys_dyeRotor = dyerotor;
        this.setpoint = setpoint;
        this.isSetpoint = true;
        addRequirements(this.sys_dyeRotor);
    }

    public DyeRotorSpin(DyeRotorSubsystem dyerotor, Current amps) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.sys_dyeRotor = dyerotor;
        this.amps = amps;
        this.isAmps = true;
        addRequirements(this.sys_dyeRotor);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (this.isSetpoint) this.sys_dyeRotor.setSpeed(this.setpoint);
        if (this.isAmps) this.sys_dyeRotor.setAmps(this.amps);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted) this.sys_dyeRotor.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
