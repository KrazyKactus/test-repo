// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants.DeployConstants;
import frc.robot.subsystems.intake.DeploySubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeAgitate extends SequentialCommandGroup {
    /** Creates a new IntakeAgitate. */
    private final DeploySubsystem sys_deploy;

    private final IntakeDeploy upCommand;
    private final IntakeDeploy downCommand;

    public IntakeAgitate(DeploySubsystem deploySubsystem) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        this.sys_deploy = deploySubsystem;
        this.upCommand = new IntakeDeploy(deploySubsystem, 0.15);
        this.downCommand = new IntakeDeploy(deploySubsystem, DeployConstants.kExtended);

        addCommands(upCommand.withTimeout(0.75), downCommand.withTimeout(0.75));
    }
}
