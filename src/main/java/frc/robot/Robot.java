// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.Rotations;

import java.util.ArrayList;
import java.util.Dictionary;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.List;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.commands.ArmToPose;
import edu.wpi.first.units.BaseUnits.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.Tracks;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Robot extends LoggedRobot {

    private Command m_autonomousCommand;
    public boolean utilizeLimelight = true;

    // Create field models.
    Field2d llestamation = new Field2d();
    Field2d robotPose = new Field2d();
    Field2d nearestPoseField = new Field2d();

    // Create robotContainer.
    private final RobotContainer robotContainer;

    public Robot() {
        // Create robot container and upload field models.
        robotContainer = new RobotContainer();
        SmartDashboard.putData("Limelight Pose", llestamation);
        SmartDashboard.putData("Robot Pose", robotPose);
        SmartDashboard.putData("Navigate Target Pose", nearestPoseField);

        // Create limelight boolean (detecting)
        for (String limelight : Constants.LIMELIGHT_NAMES) {
            SmartDashboard.putBoolean(limelight + " detecting", false);
        }

        // Start logging process.
        Logger.recordMetadata("Hotwire Project", "2026");

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new NT4Publisher());
        } else {
            setUseTiming(false);
            String logPath = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath,
            "_sim")));
        }

        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA); //TODO Upload BuildConstants?
        Logger.start();
    }

    @Override
    public void robotInit() {
        PathfindingCommand.warmupCommand().schedule();
    }

    

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        SmartDashboard.putNumber("Nearest ID", robotContainer.drivetrain.nearestId);

        // TODO Re-implement post-refactor.
        // for (Intake.Range range : Intake.Range.values()) {
          //   Boolean measurement = robotContainer.intake.getMeasurement(range);
          //   SmartDashboard.putBoolean(range.toString() + " CANrange", measurement);
        // }

        robotPose.setRobotPose(robotContainer.drivetrain.getState().Pose);

        SmartDashboard.putString("Alliance", DriverStation.getAlliance().toString());
        
        if (utilizeLimelight) {
            List<PoseEstimate> measurements = new ArrayList<PoseEstimate>();

            SwerveDriveState driveState;
            double headingDeg;
            double omegaRPS;
            PoseEstimate limelightMeasurement;

            /*
                `limelight-one` is back.
                `limelight-two` is front.
            */
            boolean detectedFlag = false;
            for (String limelight : Constants.LIMELIGHT_NAMES) {

                driveState = robotContainer.drivetrain.getState();
                headingDeg = driveState.Pose.getRotation().getDegrees();
                omegaRPS = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
                /*
                    Pipeline 0 is for the red side,
                    Pipeline 1 is for the blue side.
                */
                // LimelightHelpers.setPipelineIndex(limelight, 
                    // (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? 0 : 1
                    // ? (m_robotContainer.drivetrain.getState().Pose.getY() < 4.02 ? 0 : 2) : 
                    //   (m_robotContainer.drivetrain.getState().Pose.getY() < 4.02 ? 1 : 3)
                // );
                LimelightHelpers.setPipelineIndex(limelight, 0);
                LimelightHelpers.SetRobotOrientation(limelight, headingDeg, 0, 0, 0, 0, 0);
                limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight);

                // If the measurement is proper, record the measurement.
                if ((limelightMeasurement != null) && (limelightMeasurement.tagCount > 0) && (Math.abs(omegaRPS) < 2) && (limelightMeasurement.avgTagDist < 2.75)) {
                    measurements.add(limelightMeasurement);
                    SmartDashboard.putBoolean(limelight + " detecting", true);
                    // Enable flag.
                    detectedFlag = true;
                    robotContainer.drivetrain.addVisionMeasurement(limelightMeasurement.pose,
                      limelightMeasurement.timestampSeconds);
                    llestamation.setRobotPose(limelightMeasurement.pose);
                } else {
                    // Disable flag.
                    SmartDashboard.putBoolean(limelight + " detecting", false);
                }
            }

            // Detected logic.
            if (detectedFlag) {
                detectedFlag = false;
                for (String limelightName : Constants.LIMELIGHT_NAMES) {
                    LimelightHelpers.setLEDMode_ForceOn(limelightName);
                }
            } else {
                for (String limelightName : Constants.LIMELIGHT_NAMES) {
                    LimelightHelpers.setLEDMode_ForceOff(limelightName);
                }
            }
        }

        if (robotContainer.drivetrain.nearestPose != null) {
            nearestPoseField.setRobotPose(robotContainer.drivetrain.nearestPose);
        }
    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void disabledExit() {

    }

    @Override
    public void autonomousInit() {
        robotContainer.superstructure.targetState = Superstructure.TargetState.AUTONOMOUS;
        m_autonomousCommand = robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void autonomousExit() {

    }

    @Override
    public void teleopInit() {
        robotContainer.superstructure.targetState = Superstructure.TargetState.DEFAULT;
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void teleopExit() {

    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void testExit() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
