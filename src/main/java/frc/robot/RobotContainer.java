// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.Tracks;
import frc.robot.commands.CommandGenerator;
import frc.robot.commands.SetTargetPose;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.TargetState;
import frc.robot.subsystems.Intake.Range;

import java.lang.annotation.Target;
import java.nio.file.Path;
import java.nio.file.Paths;

public class RobotContainer {
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(Constants.MaxSpeed);

    /** Drivetrain Subsystem */
    class DriveTrain extends SwerveDriveTrain {
        public DriveTrain() {
            super(
                TunerConstants.DrivetrainConstants,
                TunerConstants.FrontLeft, TunerConstants.FrontRight,
                TunerConstants.BackLeft,  TunerConstants.BackRight
            );
        }

    }
    public final DriveTrain drivetrain = new DriveTrain();

    /** Arm Subsystem */
    public final Arm arm = new Arm();

    /** Intake Subsystem */
    public final Intake intake = new Intake();

    /** State Superstructure */
    public final Superstructure superstructure = new Superstructure(this);

    private final SendableChooser<Command> autoChooser;
        public RobotContainer() {
            SmartDashboard.putString("Auto Step", "None");
    
            registerNamedCommands();
            configureBindings();
    
            autoChooser = AutoBuilder.buildAutoChooser();
            SmartDashboard.putData("Selected Auto", autoChooser);
        }
    
        private void registerNamedCommands() {
            //TODO Register commands
            // NamedCommands.registerCommand("GoToL3", goToL3Command());
            // NamedCommands.registerCommand("EjectCoral", new InstantCommand(() -> {
            //     SmartDashboard.putString("Auto Step", "EjectCoral");
            //     superstructure.targetState = Superstructure.TargetState.EJECT;
            // }));
            // NamedCommands.registerCommand("EjectCoralReverse", ejectCoralCommand(EjectDirection.REVERSE));
            // NamedCommands.registerCommand("EjectCoralForward", ejectCoralCommand(EjectDirection.FORWARD));
            // NamedCommands.registerCommand("RemoveAlgae", removeAlgaeCommand());
            // NamedCommands.registerCommand("ArmToL3", armToL3Command());
            // NamedCommands.registerCommand("RemoveHighAlgae", armToL3Command().andThen(removeAlgaeCommand()));
            // NamedCommands.registerCommand("ArmToL2", armToL2Command());
            // NamedCommands.registerCommand("ArmToIntake", armToIntakeCommand());
            // NamedCommands.registerCommand("ArmToPunch", armToPunchCommand());
            // NamedCommands.registerCommand("CutIntake", cutIntakeCommand());
            // NamedCommands.registerCommand("ExitStart", armExitStartCommand().andThen(armToIntakeCommand()));
            // NamedCommands.registerCommand("ExitStartToL2", voltageExitStartCommand().andThen(armToL2Command()));
            // NamedCommands.registerCommand("IntakeCoral", intakeCoralCommand());
            // NamedCommands.registerCommand("ReadyScoreL3", readyScoreL3Command());
            // NamedCommands.registerCommand("ScoreL3Coral", scoreL3Command());
            // NamedCommands.registerCommand("ToDefault", new InstantCommand(() -> {
            //     SmartDashboard.putString("Auto Step", "ToDefault");
            //     superstructure.targetState = Superstructure.TargetState.DEFAULT;
            // }));
            // NamedCommands.registerCommand("ToAlgaeL2", new InstantCommand(() -> {
            //     SmartDashboard.putString("Auto Step", "ToAlgaeL2");
            //     superstructure.targetState = Superstructure.TargetState.ALGAE_L2_AUTO;
            // }));
        }
        
    private void configureBindings() {
        
        // Lower the intake.
        Constants.operator.x().whileTrue(
            new InstantCommand(() -> {
                System.out.println("Lower");
                superstructure.targetState = Superstructure.TargetState.LOWER;
            })
        ).onFalse(
            new InstantCommand(() -> {
                System.out.println("Default");
                superstructure.targetState = Superstructure.TargetState.DEFAULT;
            })
        );

        // Run intake devices.
        Constants.operator.y().whileTrue(
            new InstantCommand(() -> {
                System.out.println("Intake");
                superstructure.targetState = Superstructure.TargetState.INTAKE;
            })
        ).onFalse(
            new InstantCommand(() -> {
                System.out.println("Default");
                superstructure.targetState = Superstructure.TargetState.DEFAULT;
            })
        );

        // Run arm up.
        Constants.driver.povUp().whileTrue(
            new InstantCommand(() -> {
                System.out.println("Exit Start");
                superstructure.targetState = Superstructure.TargetState.EXIT_STARTING_POSE;
            })
        ).onFalse(
            new InstantCommand(() -> {
                System.out.println("Default");
                superstructure.targetState = Superstructure.TargetState.DEFAULT;
            })
        );
        Constants.operator.povUp().whileTrue(
            new InstantCommand(() -> {
                System.out.println("Exit Start");
                superstructure.targetState = Superstructure.TargetState.EXIT_STARTING_POSE;
            })
        ).onFalse(
            new InstantCommand(() -> {
                System.out.println("Default");
                superstructure.targetState = Superstructure.TargetState.DEFAULT;
            })
        );

        // Eject.
        Constants.driver.povRight().whileTrue(
            new InstantCommand(() -> {
                System.out.println("Eject");
                superstructure.targetState = Superstructure.TargetState.EJECT;
            })
        ).onFalse(
            new InstantCommand(() -> {
                System.out.println("Default");
                superstructure.targetState = Superstructure.TargetState.DEFAULT;
            })
        );
        Constants.operator.povRight().whileTrue(
            new InstantCommand(() -> {
                System.out.println("Eject");
                superstructure.targetState = Superstructure.TargetState.EJECT;
            })
        ).onFalse(
            new InstantCommand(() -> {
                System.out.println("Default");
                superstructure.targetState = Superstructure.TargetState.DEFAULT;
            })
        );

        // Scoring Positions
        Constants.operator.rightBumper().onTrue(
            new InstantCommand(() -> {
                System.out.println("Up-Right");
                superstructure.targetState = Superstructure.TargetState.NAVIGATE_UP_RIGHT;
            })
        );

        Constants.operator.leftBumper().onTrue(
            new InstantCommand(() -> {
                System.out.println("Up-Left");
                superstructure.targetState = Superstructure.TargetState.NAVIGATE_UP_LEFT;
            })
        );

        // Remove Algae
        Constants.operator.povDown().onTrue(
            new InstantCommand(() -> {
                superstructure.targetState = Superstructure.TargetState.NAVIGATE_ALGAE;
            })
        );
        
        Constants.operator.rightTrigger().onTrue(
            new InstantCommand(() -> {
                System.out.println("Down-Right");
                superstructure.targetState = Superstructure.TargetState.NAVIGATE_DOWN_RIGHT;
            })
        );

        Constants.operator.leftTrigger().onTrue(
            new InstantCommand(() -> {
                System.out.println("Down-Left");
                superstructure.targetState = Superstructure.TargetState.NAVIGATE_DOWN_LEFT;
            })
        );

        // Abort
        Constants.operator.back().onTrue(
            new InstantCommand(() -> {
                System.out.println("Abort");
                superstructure.targetState = Superstructure.TargetState.DEFAULT;
            })
        );

        Constants.driver.back().onTrue(
            new InstantCommand(() -> {
                System.out.println("Abort");
                superstructure.targetState = Superstructure.TargetState.DEFAULT;
            })
        );

        // Idle mode while disabled
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled()
            .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        // Driver controls
        Constants.driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        Constants.driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(
                new Rotation2d(-Constants.driver.getLeftY(), -Constants.driver.getLeftX())
            )
        ));

        // SysId routines
        Constants.driver.back().and(Constants.driver.y())
            .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));

        Constants.driver.back().and(Constants.driver.x())
            .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));

        Constants.driver.start().and(Constants.driver.y())
            .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));

        Constants.driver.start().and(Constants.driver.x())
            .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset field heading
        Constants.driver.povDown().onTrue(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric())
        );

        // Telemetry
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}