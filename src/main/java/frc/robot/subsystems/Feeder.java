// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.KrakenX60;
import frc.robot.Constants.Ports;
import frc.robot.Constants.ShuffleboardConstants;

public class Feeder extends SubsystemBase {
  public enum Speed {
    FEED(5000),
    UNSTUCK(-2000);

    private final double rpm;

    private Speed(double rpm) {
      this.rpm = rpm;
    }

    public AngularVelocity angularVelocity() {
      return RPM.of(rpm);
    }
  }

  private final TalonFX m_motor;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  public Feeder() {
    m_motor = new TalonFX(Ports.kFeeder, Ports.kRoboRioCANBus);

    final TalonFXConfiguration config = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(120))
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(Amps.of(50))
                .withSupplyCurrentLimitEnable(true))
        .withSlot0(
            new Slot0Configs()
                .withKP(0.1)
                .withKI(0)
                .withKD(0)
                .withKV(12.0 / KrakenX60.kFreeSpeed.in(RotationsPerSecond)));

    m_motor.getConfigurator().apply(config);
    SmartDashboard.putData(this);
  }

  public void set(Speed speed) {
    m_motor.setControl(
        velocityRequest.withVelocity(speed.angularVelocity()));
  }

  public void setPercentOutput(double percentOutput) {
    m_motor.setControl(
        voltageRequest.withOutput(Volts.of(percentOutput * 12.0)));
  }

  public Command feedCommand() {
    return startEnd(() -> set(Speed.FEED), () -> setPercentOutput(0));
  }

  public Command unstuckCommand() {
    return startEnd(() -> set(Speed.UNSTUCK), () -> setPercentOutput(0));
  }

  public void stop() {
    setPercentOutput(0.0);
  }

  private GenericEntry currentCommandEntry = ShuffleboardConstants.kFeederTab.add("Feeder/Current Command", "null")
      .withWidget(BuiltInWidgets.kCommand)
      .withSize(2, 1)
      .withPosition(0, 0)
      .getEntry();
  private GenericEntry rpmEntry = ShuffleboardConstants.kFeederTab.add("Feeder/RPM", 0.0)
      .withWidget(BuiltInWidgets.kTextView)
      .withSize(2, 1)
      .withPosition(2, 0)
      .getEntry();
  private GenericEntry statorCurrentEntry = ShuffleboardConstants.kFeederTab.add("Feeder/Stator Current", 0.0)
      .withWidget(BuiltInWidgets.kTextView)
      .withSize(2, 1)
      .withPosition(3, 0)
      .getEntry();
  private GenericEntry supplyCurrentEntry = ShuffleboardConstants.kFeederTab.add("Feeder/Supply Current", 0.0)
      .withWidget(BuiltInWidgets.kTextView)
      .withSize(2, 1)
      .withPosition(4, 0)
      .getEntry();

  @Override
  public void periodic() {
    currentCommandEntry.setString(getCurrentCommand() != null ? getCurrentCommand().getName() : "null");
    rpmEntry.setDouble(m_motor.getVelocity().getValue().in(RPM));
    statorCurrentEntry.setDouble(m_motor.getStatorCurrent().getValue().in(Amps));
    supplyCurrentEntry.setDouble(m_motor.getSupplyCurrent().getValue().in(Amps));
  }
}
