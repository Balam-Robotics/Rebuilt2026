// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;
import frc.util.GeometryUtil;
import frc.util.Landmarks;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveRobotShotCommand extends Command {
    private static final Angle kPositionTolerance = Degrees.of(5);
    private static final double kPositionThreshold = 0.5; // meters

    private final Swerve swerve;
    private final Supplier<Pose2d> robotPoseSupplier;

    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleRequest = new SwerveRequest.FieldCentricFacingAngle()
        .withRotationalDeadband(SwerveConstants.kPIDRotationDeadband)
        .withMaxAbsRotationalRate(SwerveConstants.kMaxRotationalRate)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
        .withHeadingPID(5, 0, 0);

    public MoveRobotShotCommand(Swerve swerve, Supplier<Pose2d> robotPoseSupplier) {
        this.swerve = swerve;
        this.robotPoseSupplier = robotPoseSupplier;
        addRequirements(swerve);
    }

    private Pose2d getBestShootingLocation() {
        // Get the target shooting location (e.g., from vision, landmarks, or strategy)
        // For now, return the hub position at a reasonable distance
        final Translation2d hubPosition = Landmarks.hubPosition();
        // Position the robot 2 meters away from the hub
        final Translation2d robotTargetPosition = hubPosition.minus(
            new Translation2d(2.0, 0).rotateBy(getDirectionToHub())
        );
        return new Pose2d(robotTargetPosition, getDirectionToHub());
    }

    private Rotation2d getDirectionToHub() {
        final Translation2d hubPosition = Landmarks.hubPosition();
        final Translation2d robotPosition = swerve.getState().Pose.getTranslation();
        final Rotation2d hubDirectionInBlueAlliancePerspective = hubPosition.minus(robotPosition).getAngle();
        final Rotation2d hubDirectionInOperatorPerspective = hubDirectionInBlueAlliancePerspective
            .rotateBy(swerve.getOperatorForwardDirection());
        return hubDirectionInOperatorPerspective;
    }

    private boolean isAtBestShootingLocation() {
        final Pose2d targetPose = getBestShootingLocation();
        final Pose2d currentPose = robotPoseSupplier.get();
        
        // Check if position is within threshold
        final double distanceToTarget = currentPose.getTranslation()
            .getDistance(targetPose.getTranslation());
        
        // Check if heading is within tolerance
        final boolean headingAligned = GeometryUtil.isNear(
            targetPose.getRotation(), 
            currentPose.getRotation(), 
            kPositionTolerance
        );
        
        return distanceToTarget < kPositionThreshold && headingAligned;
    }

    private Translation2d getVelocityTowardTarget() {
        final Pose2d targetPose = getBestShootingLocation();
        final Translation2d currentPosition = robotPoseSupplier.get().getTranslation();
        
        // Calculate direction to target
        final Translation2d displacement = targetPose.getTranslation().minus(currentPosition);
        final double distance = displacement.getNorm();
        
        if (distance < 0.1) {
            return new Translation2d(); // Stop if very close
        }
        
        // Normalize and scale by max speed
        final Translation2d direction = displacement.div(distance);
        final double speedFactor = Math.min(1.0, distance / 2.0); // Slow down as we approach
        
        return direction.times(SwerveConstants.kMaxSpeed.in(edu.wpi.first.units.Units.MetersPerSecond) * speedFactor);
    }

    @Override
    public void execute() {
        final Translation2d velocity = getVelocityTowardTarget();
        final Rotation2d targetHeading = getBestShootingLocation().getRotation();
        
        swerve.setControl(
            fieldCentricFacingAngleRequest
                .withVelocityX(velocity.getX())
                .withVelocityY(velocity.getY())
                .withTargetDirection(targetHeading)
        );
    }

    @Override
    public boolean isFinished() {
        return isAtBestShootingLocation();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot's movement when the command ends
        swerve.setControl(
            fieldCentricFacingAngleRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withTargetDirection(swerve.getState().Pose.getRotation())
        );
    }
}
