// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurretAngleSet extends Command {
    /** Creates a new TurretAngleSet. */
    private final TurretSubsystem sys_turret;

    private final double setpoint;
    private final double velocity;

    public TurretAngleSet(TurretSubsystem turret, DoubleSupplier setpoint, DoubleSupplier velocity) {
        this.sys_turret = turret;
        this.setpoint = setpoint.getAsDouble();
        this.velocity = velocity.getAsDouble();
        addRequirements(sys_turret);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        sys_turret.setAngle(setpoint, velocity);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted) this.sys_turret.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
