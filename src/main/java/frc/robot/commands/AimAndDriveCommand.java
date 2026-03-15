// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;
import frc.util.DriveInputSmoother;
import frc.util.GeometryUtil;
import frc.util.ManualDriveInput;
import frc.util.Landmarks;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimAndDriveCommand extends Command {
  private static final Angle kAimTolerance = Degrees.of(5);

  private final Swerve swerve;
  private final DriveInputSmoother inputSmoother;

  private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleRequest = new SwerveRequest.FieldCentricFacingAngle()
      .withRotationalDeadband(SwerveConstants.kPIDRotationDeadband)
      .withMaxAbsRotationalRate(SwerveConstants.kMaxRotationalRate)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withSteerRequestType(SteerRequestType.MotionMagicExpo)
      .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
      .withHeadingPID(5, 0, 0);

  public AimAndDriveCommand(
      Swerve swerve,
      DoubleSupplier forwardInput,
      DoubleSupplier leftInput) {
    this.swerve = swerve;
    this.inputSmoother = new DriveInputSmoother(forwardInput, leftInput);
    addRequirements(swerve);
  }

  public AimAndDriveCommand(Swerve swerve) {
    this(swerve, () -> 0.0, () -> 0.0);
  }

  public boolean isAimed() {
    final Rotation2d targetHeading = fieldCentricFacingAngleRequest.TargetDirection;
    final Rotation2d currentHedingInBlueAlliancePerspective = swerve.getState().Pose.getRotation();
    final Rotation2d currentHeadingInOperatorPerspective = currentHedingInBlueAlliancePerspective
        .rotateBy(swerve.getOperatorForwardDirection());
    return GeometryUtil.isNear(targetHeading, currentHeadingInOperatorPerspective, kAimTolerance);
  }

  private Rotation2d getDirectionToHub() {
    final Translation2d hubPosition = Landmarks.hubPosition();
    final Translation2d robotPosition = swerve.getState().Pose.getTranslation();
    final Rotation2d hubDirectionInBlueAlliancePerspective = hubPosition.minus(robotPosition).getAngle();
    final Rotation2d hubDirectionInOperatorPerspective = hubDirectionInBlueAlliancePerspective
        .rotateBy(swerve.getOperatorForwardDirection());
    return hubDirectionInOperatorPerspective;
  }

  @Override
  public void execute() {
    final ManualDriveInput input = inputSmoother.getSmoothedInput();
    swerve.setControl(
        fieldCentricFacingAngleRequest
            .withVelocityX(SwerveConstants.kMaxSpeed.times(input.forward))
            .withVelocityY(SwerveConstants.kMaxSpeed.times(input.left))
            .withTargetDirection(getDirectionToHub()));
  }

  @Override
  public boolean isFinished() {
    return isAimed();
  }
}
