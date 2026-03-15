// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;

import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.util.SwerveTelemetry;

import frc.robot.Constants.SwerveConstants;

public class RobotContainer {
    private final Swerve swerve = new Swerve();
    private final Limelight limelight_front = new Limelight("limelight-front");
    private final Limelight limelight_back = new Limelight("limelight-back");

    private final SwerveTelemetry m_swerveTelemetry = new SwerveTelemetry(SwerveConstants.kMaxSpeed.in(MetersPerSecond));

    private final CommandXboxController m_controller = new CommandXboxController(0);

    private final AutoRoutines m_autoRoutines = new AutoRoutines(
        swerve
    );

    public RobotContainer() {
        configureBindings();
        m_autoRoutines.configure();
        swerve.registerTelemetry(m_swerveTelemetry::telemeterize);
    }

    private void configureBindings() {
        configureManualDriveBindings();
        limelight_front.setDefaultCommand(updateVisionCommand(limelight_front));
        limelight_back.setDefaultCommand(updateVisionCommand(limelight_back));
    }

    private void configureManualDriveBindings() {
        final ManualDriveCommand manualDriveCommand = new ManualDriveCommand(
            swerve,
            () -> -m_controller.getLeftY(),
            () -> -m_controller.getLeftX(),
            () -> -m_controller.getRightX()
        );
        swerve.setDefaultCommand(manualDriveCommand);

        m_controller.a().onTrue(manualDriveCommand);
    }

    private Command updateVisionCommand(Limelight limelight) {
        return limelight.run(() -> {
            final Pose2d currentRobotPose = swerve.getState().Pose;
            final Optional<Limelight.Measurement> measurement = limelight.getMeasurement(currentRobotPose);
            measurement.ifPresent(m -> {
                swerve.addVisionMeasurement(
                    m.poseEstiamte.pose,
                    m.poseEstiamte.timestampSeconds,
                    m.standardDeviations
                    );
            });
        }).ignoringDisable(true);
    }
}
