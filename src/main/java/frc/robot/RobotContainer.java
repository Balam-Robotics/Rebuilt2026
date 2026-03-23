// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.SubsystemCommands;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Hanger;
import frc.util.SwerveTelemetry;

import frc.robot.Constants.SwerveConstants;

public class RobotContainer {
    private final Swerve swerve = new Swerve();

    private final Intake intake = new Intake();
    private final Floor floor = new Floor();
    private final Feeder feeder = new Feeder();
    private final Shooter shooter = new Shooter();
    private final Hood hood = new Hood();
    private final Hanger hanger = new Hanger();

    private final Limelight limelight_front = new Limelight("limelight-front");
    private final Limelight limelight_back = new Limelight("limelight");

    private final SwerveTelemetry m_swerveTelemetry = new SwerveTelemetry(
            SwerveConstants.kMaxSpeed.in(MetersPerSecond));

    private final CommandXboxController m_controller = new CommandXboxController(0);

    private final AutoRoutines m_autoRoutines = new AutoRoutines(
            swerve,
            intake,
            floor,
            feeder,
            shooter,
            hood,
            hanger,
            limelight_front);

    private final SubsystemCommands m_subsystemCommands = new SubsystemCommands(
            swerve,
            intake,
            floor,
            feeder,
            shooter,
            hood,
            hanger,
            () -> -m_controller.getLeftY(),
            () -> -m_controller.getLeftX());

    public RobotContainer() {
        configureBindings();
        m_autoRoutines.configure();
        swerve.registerTelemetry(m_swerveTelemetry::telemeterize);
    }

    private void configureBindings() {
        configureManualDriveBindings();
        limelight_front.setDefaultCommand(updateVisionCommand(limelight_front));
        limelight_back.setDefaultCommand(updateVisionCommand(limelight_back));

        RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop())
                .onTrue(intake.homingCommand())
                .onTrue(hanger.homingCommand());

        m_controller.rightTrigger().whileTrue(m_subsystemCommands.aimAndShoot());
        m_controller.rightBumper().whileTrue(m_subsystemCommands.shootManually());
        m_controller.leftTrigger().whileTrue(intake.intakeCommand());
        m_controller.leftBumper().onTrue(intake.runOnce(() -> intake.set(Intake.Position.STOWED)));

        m_controller.b().whileTrue(m_subsystemCommands.unstuck());
        m_controller.y().onTrue(hanger.positionCommand(Hanger.Position.HANGING));
        m_controller.a().onTrue(hanger.positionCommand(Hanger.Position.HUNG));
        m_controller.start().onTrue(swerve.flipRobotMode());
    }

    private void configureManualDriveBindings() {
        final ManualDriveCommand manualDriveCommand = new ManualDriveCommand(
                swerve,
                () -> -m_controller.getLeftY(),
                () -> -m_controller.getLeftX(),
                () -> -m_controller.getRightX());
        swerve.setDefaultCommand(manualDriveCommand);

        m_controller.povDown().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.k180deg)));
        m_controller.povRight()
                .onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kCW_90deg)));
        m_controller.povLeft()
                .onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kCCW_90deg)));
        m_controller.povUp().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kZero)));
        m_controller.x().onTrue(Commands.runOnce(() -> manualDriveCommand.seedFieldCentric()));
    }

    private Command updateVisionCommand() {
        return Commands.run(() -> {
            final Pose2d currentRobotPose = swerve.getState().Pose;

            final Optional<Limelight.Measurement> frontMeasurement = limelight_front.getMeasurement(currentRobotPose);
            frontMeasurement.ifPresent(m -> {
                swerve.addVisionMeasurement(
                        m.poseEstimate.pose,
                        m.poseEstimate.timestampSeconds,
                        m.standardDeviations);
            });

            final Optional<Limelight.Measurement> backMeasurement = limelight_back.getMeasurement(currentRobotPose);
            backMeasurement.ifPresent(m -> {
                swerve.addVisionMeasurement(
                        m.poseEstimate.pose,
                        m.poseEstimate.timestampSeconds,
                        m.standardDeviations);
            });

        }, limelight_back, limelight_front).ignoringDisable(true);
    }

    private Command updateVisionCommand(Limelight limelight) {
        return limelight.run(()-> {
                final Pose2d currentRobotPose = swerve.getState().Pose;
                final Optional<Limelight.Measurement> measuremnet = limelight.getMeasurement(currentRobotPose);
                measuremnet.ifPresent(m -> {
                        swerve.addVisionMeasurement(
                                m.poseEstimate.pose,
                                m.poseEstimate.timestampSeconds,
                                m.standardDeviations
                                );
                });
        }).ignoringDisable(true);
    }
}
