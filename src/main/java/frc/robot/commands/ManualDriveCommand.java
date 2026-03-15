// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.util.DriveInputSmoother;
import frc.util.ManualDriveInput;
import frc.util.Stopwatch;

import frc.robot.Constants.SwerveConstants;

public class ManualDriveCommand extends Command {
  private enum State {
    IDLING,
    DRIVING_WITH_MANUAL_ROTATION,
    DRIVING_WITH_LOCKED_HEADING,
    DRIVING_WITH_ROBOT_CENTRIC_CONTROL
  }

  private static final Time kHEADING_LOCK_TIMEOUT = Seconds.of(0.25);

  private final Swerve swerve;
  private final DriveInputSmoother inputSmoother;
  private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

  private final SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withSteerRequestType(SteerRequestType.MotionMagicExpo)
      .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

  private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleRequest = new SwerveRequest.FieldCentricFacingAngle()
      .withRotationalDeadband(SwerveConstants.kPIDRotationDeadband)
      .withMaxAbsRotationalRate(SwerveConstants.kMaxRotationalRate)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withSteerRequestType(SteerRequestType.MotionMagicExpo)
      .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
      .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
      .withHeadingPID(5, 0, 0);

  private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  private State currentState = State.IDLING;
  private Optional<Rotation2d> lockHeading = Optional.empty();
  private Stopwatch headingLockStopwatch = new Stopwatch();
  private ManualDriveInput previousInput = new ManualDriveInput();

  public ManualDriveCommand(
    Swerve swerve,
    DoubleSupplier forwardInput,
    DoubleSupplier leftInput,
    DoubleSupplier rotationInput
  ) {
    this.swerve = swerve;
    this.inputSmoother = new DriveInputSmoother(forwardInput, leftInput, rotationInput);
    addRequirements(swerve);
  }

  public void seedFieldCentric() {
    initialize();
    swerve.seedFieldCentric();
  }

  public void setLockedHeading(Rotation2d heading) {
    lockHeading = Optional.of(heading);
    headingLockStopwatch.reset();
  }

  private void setLockedHeadingToCurrent() {
    final Rotation2d headingInBlueAlliancePerspective = swerve.getState().Pose.getRotation();
    final Rotation2d headingInOperatorPerspective = headingInBlueAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection());
    setLockedHeading(headingInOperatorPerspective);
  }

  private void lockHeadingIfRotationStopped(ManualDriveInput input) {
    if (input.hasRotation()) {
      headingLockStopwatch.reset();
      lockHeading = Optional.empty();
    } else {
      headingLockStopwatch.startIfNotRunning();
      if (headingLockStopwatch.elapsedTime().gt(kHEADING_LOCK_TIMEOUT)) {
        setLockedHeadingToCurrent();
      }
    }
  }

  @Override
  public void initialize() {
    currentState = State.IDLING;
    lockHeading = Optional.empty();
    headingLockStopwatch.reset();
    previousInput = new ManualDriveInput();
  }

  @Override
  public void execute() {
    final ManualDriveInput input = inputSmoother.getSmoothedInput();

    // check for centric in swerve
    if (swerve.m_fieldRelative) {
      currentState = State.DRIVING_WITH_ROBOT_CENTRIC_CONTROL;
    } else if (input.hasRotation()) {
      currentState = State.DRIVING_WITH_MANUAL_ROTATION;
    } else if (input.hasTranslation()) {
      currentState = lockHeading.isPresent() ? State.DRIVING_WITH_LOCKED_HEADING : State.DRIVING_WITH_MANUAL_ROTATION;
    } else if (previousInput.hasRotation() || previousInput.hasTranslation()) {
      currentState = State.IDLING;
    }
    previousInput = input;

    switch (currentState) {
      case IDLING:
        swerve.setControl(idleRequest);
        break;
      case DRIVING_WITH_MANUAL_ROTATION:
      lockHeadingIfRotationStopped(input);
        swerve.setControl(fieldCentricRequest
          .withVelocityX(SwerveConstants.kMaxSpeed.times(input.forward))
          .withVelocityY(SwerveConstants.kMaxSpeed.times(input.left))
          .withRotationalRate(SwerveConstants.kMaxRotationalRate.times(input.rotation)));
        break;
      case DRIVING_WITH_LOCKED_HEADING:
        swerve.setControl(fieldCentricFacingAngleRequest
          .withVelocityX(SwerveConstants.kMaxSpeed.times(input.forward))
          .withVelocityY(SwerveConstants.kMaxSpeed.times(input.left))
          .withTargetDirection(lockHeading.get()));
        break;
      case DRIVING_WITH_ROBOT_CENTRIC_CONTROL:
        swerve.setControl(robotCentricRequest
          .withVelocityX(SwerveConstants.kMaxSpeed.times(input.forward))
          .withVelocityY(SwerveConstants.kMaxSpeed.times(input.left))
          .withRotationalRate(SwerveConstants.kMaxRotationalRate.times(input.rotation)));
        break;
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
