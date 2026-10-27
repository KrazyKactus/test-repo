// Copyright 2021-2025 FRC 6328
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

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DyeRotorConstants;
import frc.robot.Constants.IntakeConstants.DeployConstants;
import frc.robot.Constants.IntakeConstants.FieldConstants;
import frc.robot.Constants.IntakeConstants.RollerConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DyeRotorSpin;
import frc.robot.commands.HoodAimShoot;
import frc.robot.commands.HoodIdle;
import frc.robot.commands.IntakeAgitate;
import frc.robot.commands.IntakeDeploy;
import frc.robot.commands.IntakeSpin;
import frc.robot.commands.ShootOnTheMove;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DyeRotorSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.DeploySubsystem;
import frc.robot.subsystems.intake.RollerSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Vision vision;
    private final Drive drive;
    private FlywheelSubsystem flywheelSubsystem = new FlywheelSubsystem();
    private HoodSubsystem hoodSubsystem;
    private DyeRotorSubsystem dyeRotorSubsystem;
    private DeploySubsystem deploySubsystem;
    private RollerSubsystem rollerSubsystem;
    private TurretSubsystem turretSubsystem;
    private SwerveDriveSimulation driveSimulation = null;

    private Command deployAtZeroCommand;
    private Command deployAtExtendCommand;
    private Command intakeCommand;
    private Command intakeStopCommand;
    private Command shootCommand;
    private Command turretShootCommand;

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);
    private boolean hasPressed = false;

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight),
                        (robotPose) -> {
                        });
                vision = new Vision(
                        drive,
                        new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                        new VisionIOPhotonVision(VisionConstants.rightCamera,
                                VisionConstants.robotToCameraRight),
                        new VisionIOPhotonVision(VisionConstants.leftCamera,
                                VisionConstants.robotToCameraLeft));

                hoodSubsystem = new HoodSubsystem();
                dyeRotorSubsystem = new DyeRotorSubsystem();
                deploySubsystem = new DeploySubsystem();
                rollerSubsystem = new RollerSubsystem();
                turretSubsystem = new TurretSubsystem();

                deployAtZeroCommand = new IntakeDeploy(deploySubsystem, DeployConstants.kZero);
                deployAtExtendCommand = new IntakeDeploy(deploySubsystem, DeployConstants.kExtended);
                intakeCommand = new IntakeSpin(rollerSubsystem, RollerConstants.kForwardSpeed);
                intakeStopCommand = new IntakeSpin(rollerSubsystem, 0);

                turretShootCommand = new ShootOnTheMove(
                        drive,
                        turretSubsystem,
                        flywheelSubsystem,
                        hoodSubsystem,
                        DriverStation.getAlliance().isPresent()
                                ? (DriverStation.getAlliance().get() == Alliance.Red
                                        ? FieldConstants.redHub
                                        : FieldConstants.bluehub)
                                : new Translation2d(0, 0));

                shootCommand = new ParallelCommandGroup(
                        new HoodAimShoot(
                                hoodSubsystem,
                                flywheelSubsystem,
                                drive.calculateDistanceToPoint(
                                        DriverStation.getAlliance()
                                                .get() == Alliance.Red
                                                        ? FieldConstants.redHub
                                                        : FieldConstants.bluehub)),
                        // new HoodAimShoot(hoodSubsystem, flywheelSubsystem, 3),
                        new SequentialCommandGroup(
                                new WaitCommand(0.5),
                                new DyeRotorSpin(dyeRotorSubsystem,
                                        DyeRotorConstants.kForwardSpeed)),
                        new SequentialCommandGroup(
                                new WaitCommand(2),
                                new ParallelCommandGroup(
                                        new IntakeAgitate(deploySubsystem)
                                                .repeatedly(),
                                        new IntakeSpin(rollerSubsystem,
                                                RollerConstants.kForwardSpeed))));

                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                driveSimulation = new SwerveDriveSimulation(Drive.getMapleSimConfig(),
                        new Pose2d(3, 3, new Rotation2d()));
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                drive = new Drive(
                        new GyroIOSim(driveSimulation.getGyroSimulation()),
                        new ModuleIOSim(driveSimulation.getModules()[0]),
                        new ModuleIOSim(driveSimulation.getModules()[1]),
                        new ModuleIOSim(driveSimulation.getModules()[2]),
                        new ModuleIOSim(driveSimulation.getModules()[3]),
                        driveSimulation::setSimulationWorldPose);

                vision = new Vision(
                        drive,
                        new VisionIOPhotonVisionSim(
                                VisionConstants.rightCamera,
                                VisionConstants.robotToCameraRight,
                                driveSimulation::getSimulatedDriveTrainPose),
                        new VisionIOPhotonVisionSim(
                                VisionConstants.leftCamera,
                                VisionConstants.robotToCameraLeft,
                                driveSimulation::getSimulatedDriveTrainPose));

                break;

            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        (robotPose) -> {
                        });
                vision = new Vision(drive, new VisionIO() {
                }, new VisionIO() {
                });

                break;
        }

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Configure the button bindings
        configureButtonBindings();
        registerNamedCommands();
        autoChooserAddOptions();
    }

    // registers all named commands for pathplanner
    private void registerNamedCommands() {
        switch (Constants.currentMode) {
            case REAL:
                NamedCommands.registerCommand("shoot", shootCommand);
                NamedCommands.registerCommand("deployAtZero", deployAtZeroCommand);
                NamedCommands.registerCommand("deployAtExtend", deployAtExtendCommand);
                NamedCommands.registerCommand("intake", intakeCommand);
                NamedCommands.registerCommand("stopIntake", intakeStopCommand);
                break;

            default:
                break;
        }
    }

    private void autoChooserAddOptions() {
        switch (Constants.currentMode) {
            case REAL:
                // Side options
                autoChooser.addOption("Right Side Single Cycle", new PathPlannerAuto("OneCycleAuto"));
                autoChooser.addOption("Left Side Single Cycle",
                        new PathPlannerAuto("OneCycleAuto", true));
                autoChooser.addOption("Right Side 1 1/2 Cycle",
                        new PathPlannerAuto("OneAndAHalfCycleAuto"));
                autoChooser.addOption("Left Side 1 1/2 Cycle",
                        new PathPlannerAuto("OneAndAHalfCycleAuto", true));
                autoChooser.addOption("Right Side Two Cycle", new PathPlannerAuto("TwoCycleAuto"));
                autoChooser.addOption("Left Side Two Cycle", new PathPlannerAuto("TwoCycleAuto", true));
                autoChooser.addOption("Right Side Preload Cycle",
                        new PathPlannerAuto("PreloadedSideAuto"));
                autoChooser.addOption("Left Side Preload Cycle",
                        new PathPlannerAuto("PreloadedSideAuto", true));

                // Center options
                autoChooser.addOption("Center Depot Cycle", new PathPlannerAuto("CenterDepotAuto"));
                autoChooser.addOption("Center Preload Cycle",
                        new PathPlannerAuto("PreloadedCenterAuto"));

                break;

            case SIM:
                // Side options with no named commands
                autoChooser.addOption("Right Side Single Cycle",
                        new PathPlannerAuto("OneCycleAutoSim"));

                break;

            default:
                break;
        }

        // Set up SysId routines
        autoChooser.addOption("Drive Wheel Radius Characterization",
                DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption("Drive Simple FF Characterization",
                DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("Drive SysId (Dynamic Forward)",
                drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Drive SysId (Dynamic Reverse)",
                drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a
     * {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive,
                () -> -controller.getLeftY() * (hasPressed ? 0.3 : 1),
                () -> -controller.getLeftX() * (hasPressed ? 0.3 : 1),
                () -> -controller.getRightX()));

        // Reset gyro / odometry
        final Runnable resetOdometry = Constants.currentMode == Constants.Mode.SIM
                ? () -> drive.resetOdometry(driveSimulation.getSimulatedDriveTrainPose())
                : () -> drive.resetOdometry(
                        new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));
        controller.start().onTrue(Commands.runOnce(resetOdometry).ignoringDisable(true));

        switch (Constants.currentMode) {
            case REAL:
                hoodSubsystem.setDefaultCommand(new HoodIdle(hoodSubsystem));
                deploySubsystem.setDefaultCommand(
                        new IntakeDeploy(deploySubsystem, DeployConstants.kExtended));

                // run rollers in an intaking direction
                controller.leftTrigger().whileTrue(
                        new IntakeSpin(rollerSubsystem, RollerConstants.kForwardSpeed));

                // run rollers in an reverse direction
                controller.x().whileTrue(
                        new IntakeSpin(rollerSubsystem, RollerConstants.kReverseSpeed));

                // Extend and Retract Intake
                controller.y().onTrue(new IntakeDeploy(deploySubsystem, DeployConstants.kExtended));

                controller.a().onTrue(new IntakeDeploy(deploySubsystem, DeployConstants.kZero));

                controller.povLeft().whileTrue(new DyeRotorSpin(dyeRotorSubsystem,
                        DyeRotorConstants.kReverseAmperage));

                controller
                        .povDown()
                        .whileTrue(new ParallelCommandGroup(
                                new IntakeDeploy(deploySubsystem, 0.18),
                                new IntakeSpin(rollerSubsystem,
                                        RollerConstants.kForwardSpeed)));

                controller.b().onTrue(Commands.runOnce(() -> hasPressed = !hasPressed));

                break;

            default:
                break;
        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void resetSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM)
            return;

        drive.resetOdometry(new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void updateSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM)
            return;

        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput("FieldSimulation/Fuel",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    }
}
