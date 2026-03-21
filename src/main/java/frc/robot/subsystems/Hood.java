// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Ports;
import frc.robot.Constants.ShuffleboardConstants;

public class Hood extends SubsystemBase {
  private static final Distance kServoLength = Millimeters.of(100);
  private static final LinearVelocity kMaxServoSpeed = Millimeters.of(20).per(Second);
  private static final double kMinPosition = 0.01;
  private static final double kMaxPosition = 0.77;
  private static final double kPositionTolerance = 0.01;

  private final Servo m_leftServo;
  private final Servo m_rightServo;

  private double currentPosition = 0.5;
  private double targetPosition = 0.5;
  private Time lastUpdateTime = Seconds.of(0);

  public Hood() {
    m_leftServo = new Servo(Ports.kHoodLeftServo);
    m_rightServo = new Servo(Ports.kHoodRightServo);

    m_leftServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
    m_rightServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);

    setPosition(currentPosition);
    SmartDashboard.putData(this);
  }

  public void setPosition(double position) {
    final double clampedPosition = MathUtil.clamp(position, kMinPosition, kMaxPosition);
    m_leftServo.set(clampedPosition);
    m_rightServo.set(clampedPosition);
    targetPosition = clampedPosition;
  }

  public Command positionCommand(double position) {
    return runOnce(() -> setPosition(position))
        .andThen(Commands.waitUntil(this::isPositionWithinTolerance));
  }

  public boolean isPositionWithinTolerance() {
    return MathUtil.isNear(kMaxPosition, currentPosition, kPositionTolerance);
  }

  private void updateCurrentPosition() {
    final Time currentTime = Seconds.of(Timer.getFPGATimestamp());
    final Time elapsedTime = currentTime.minus(lastUpdateTime);
    lastUpdateTime = currentTime;

    if (isPositionWithinTolerance()) {
      currentPosition = targetPosition;
      return;
    }

    final Distance maxDistanceTraveled = kMaxServoSpeed.times(elapsedTime);
    final double maxPercentageTraveled = maxDistanceTraveled.div(kServoLength).in(Value);
    currentPosition = targetPosition > currentPosition
        ? Math.min(targetPosition, currentPosition + maxPercentageTraveled)
        : Math.max(targetPosition, currentPosition - maxPercentageTraveled);
  }

    private GenericEntry currentCommandEntry = ShuffleboardConstants.kHoodTab.add("Current Command", 0.0)
        .withWidget(BuiltInWidgets.kCommand)
        .withSize(2, 1)
        .withPosition(0, 0)
        .getEntry();
    private GenericEntry currentPositionEntry = ShuffleboardConstants.kHoodTab.add("Current Position", 0.0)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withSize(2, 1)
        .withPosition(2, 0)
        .getEntry();
    private GenericEntry targetPositionEntry = ShuffleboardConstants.kHoodTab.add("Target Position", 0.0)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withSize(2, 1)
        .withPosition(4, 0)
        .getEntry();

    // Left servo information
    private GenericEntry leftServoAngleEntry = ShuffleboardConstants.kHoodTab.add("Left Servo Angle", 0.0)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withSize(2, 1)
        .withPosition(0, 2)
        .getEntry();
    private GenericEntry leftServoPositionEntry = ShuffleboardConstants.kHoodTab.add("Left Servo Position", 0.0)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withSize(2, 1)
        .withPosition(2, 2)
        .getEntry();
    private GenericEntry leftServoChannelEntry = ShuffleboardConstants.kHoodTab.add("Left Servo Channel", 0.0)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withSize(2, 1)
        .withPosition(4, 2)
        .getEntry();

    // Right servo information
    private GenericEntry rightServoAngleEntry = ShuffleboardConstants.kHoodTab.add("Right Servo Angle", 0.0)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withSize(2, 1)
        .withPosition(0, 3)
        .getEntry();
    private GenericEntry rightServoPositionEntry = ShuffleboardConstants.kHoodTab.add("Right Servo Position", 0.0)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withSize(2, 1)
        .withPosition(2, 3)
        .getEntry();
    private GenericEntry rightServoChannelEntry = ShuffleboardConstants.kHoodTab.add("Right Servo Channel", 0.0)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withSize(2, 1)
        .withPosition(4, 3)
        .getEntry();

  @Override
  public void periodic() {
    updateCurrentPosition();

    currentCommandEntry.setString(getCurrentCommand() != null ? getCurrentCommand().getName() : "null");
    currentPositionEntry.setDouble(currentPosition);
    targetPositionEntry.setDouble(targetPosition);
    leftServoAngleEntry.setDouble(m_leftServo.getAngle());
    leftServoPositionEntry.setDouble(m_leftServo.get());
    leftServoChannelEntry.setDouble(m_leftServo.getChannel());
    rightServoAngleEntry.setDouble(m_rightServo.getAngle());
    rightServoPositionEntry.setDouble(m_rightServo.get());
    rightServoChannelEntry.setDouble(m_rightServo.getChannel());
  }
}
