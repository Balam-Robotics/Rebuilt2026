// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Ports;

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
      return ;
    }

    final Distance maxDistanceTraveled = kMaxServoSpeed.times(elapsedTime);
    final double maxPercentageTraveled = maxDistanceTraveled.div(kServoLength).in(Value);
    currentPosition = targetPosition > currentPosition
      ? Math.min(targetPosition, currentPosition + maxPercentageTraveled)
      : Math.max(targetPosition, currentPosition - maxPercentageTraveled);
  }

  @Override
  public void periodic() {
    updateCurrentPosition();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Current Position", () -> currentPosition, null);
        builder.addDoubleProperty("Target Position", () -> targetPosition, value -> setPosition(value));
  }
}
