// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.KrakenX60;
import frc.robot.Constants.Ports;
import frc.robot.Constants.ShuffleboardConstants;


public class Hanger extends SubsystemBase {
  public enum Position {
    HOMED(0),
    EXTEND_HOPPER(2),
    HANGING(6),
    HUNG(0.2);

    private final double inches;

    private Position(double inches) {
      this.inches = inches;
    }

    public Angle motorAngle() {
      final Measure<AngleUnit> angleMeasure = Inches.of(inches).divideRatio(kHangerExtensionPerMotorAngle);
      return Rotations.of(angleMeasure.in(Rotations));
    }
  }

  private static final Per<DistanceUnit, AngleUnit> kHangerExtensionPerMotorAngle = Inches.of(6).div(Rotations.of(142));
  private static final Distance kExtensionTolerance = Inches.of(1);

  private final TalonFX m_motor;
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private boolean isHomed = false;

  public Hanger() {
    m_motor = new TalonFX(Ports.kHanger, Ports.kRoboRioCANBus);

    final TalonFXConfiguration config = new TalonFXConfiguration()
      .withMotorOutput(
        new MotorOutputConfigs()
          .withInverted(InvertedValue.Clockwise_Positive)
          .withNeutralMode(NeutralModeValue.Brake) 
      )
      .withCurrentLimits(
        new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Amps.of(20))
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(Amps.of(70))
        .withSupplyCurrentLimitEnable(true) 
      )
      .withMotionMagic(
        new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(KrakenX60.kFreeSpeed)
        .withMotionMagicAcceleration(KrakenX60.kFreeSpeed.per(Second)) 
      )
      .withSlot0(
        new Slot0Configs()
        .withKP(10)
        .withKI(0)
        .withKD(0)
        .withKV(12.0 / KrakenX60.kFreeSpeed.in(RotationsPerSecond))
      );

      m_motor.getConfigurator().apply(config);
      SmartDashboard.putData(this);
  }

  public void set(Position position) {
    m_motor.setControl(
      motionMagicRequest.withPosition(position.motorAngle())
    );
  }

  public void setPercentOutput(double percentOutput) {
    m_motor.setControl(
      voltageRequest.withOutput(Volts.of(percentOutput * 12.0))
    );
  }

  public Command positionCommand(Position position) {
    return runOnce(() -> set(position)).andThen(Commands.waitUntil(this::isExtensionWithinTolerance));
  }

  public Command homingCommand() {
    return Commands.sequence(
      runOnce(() -> setPercentOutput(-0.05)),
      Commands.waitUntil(() -> m_motor.getSupplyCurrent().getValue().in(Amps) > 0.4),
      runOnce(() -> {
        m_motor.setPosition(Position.HOMED.motorAngle());
        isHomed = true;
        set(Position.EXTEND_HOPPER);
      })
    )
    .unless(() -> isHomed)
    .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public Command extendHopperCommand() {
    return runOnce(() -> set(Position.EXTEND_HOPPER));
  }

  public boolean isHomed() {
    return isHomed;
  }

  private boolean isExtensionWithinTolerance() {
    final Distance currentExtension = motorAngleToExtension(m_motor.getPosition().getValue());
    final Distance targetExtension = motorAngleToExtension(motionMagicRequest.getPositionMeasure());
    return currentExtension.isNear(targetExtension, kExtensionTolerance);
  }

  private Distance motorAngleToExtension(Angle motorAngle) {
    final Measure<DistanceUnit> extensionMeasure = motorAngle.timesRatio(kHangerExtensionPerMotorAngle);
    return Inches.of(extensionMeasure.in(Inches));
  }

  private GenericEntry currentCommandEntry = ShuffleboardConstants.kHangerTab.add("Hanger/Current Command", 0.0)
      .withWidget(BuiltInWidgets.kCommand)
      .withSize(2, 1)
      .withPosition(0, 0)
      .getEntry();
  private GenericEntry extensionEntry = ShuffleboardConstants.kHangerTab.add("Hanger/Extension (inches)", 0.0)
      .withWidget(BuiltInWidgets.kTextView)
      .withSize(2, 1)
      .withPosition(2, 0)
      .getEntry();
  private GenericEntry supplyCurrentEntry = ShuffleboardConstants.kHangerTab.add("Hanger/Supply Current", 0.0)
      .withWidget(BuiltInWidgets.kTextView)
      .withSize(2, 1)
      .withPosition(4, 0)
      .getEntry();

  @Override
  public void periodic() {
    currentCommandEntry.setString(getCurrentCommand() != null ? getCurrentCommand().getName() : "null");
    extensionEntry.setDouble(motorAngleToExtension(m_motor.getPosition().getValue()).in(Inches));
    supplyCurrentEntry.setDouble(m_motor.getSupplyCurrent().getValue().in(Amps));
  }
}
