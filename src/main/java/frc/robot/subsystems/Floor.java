// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Ports;
import frc.robot.Constants.ShuffleboardConstants;

public class Floor extends SubsystemBase {
  public enum Speed {
    STOP(0),
    FEED(0.83);

    private final double percentOutput;

    private Speed(double percentOutput) {
      this.percentOutput = percentOutput;
    }

    public Voltage voltage() {
      return Volts.of(percentOutput * 12.0);
    }
  }

  private final TalonFX m_motor;
  private final VoltageOut voltageRequest = new VoltageOut(0);

  public Floor() {
    m_motor = new TalonFX(Ports.kFloor, Ports.kRoboRioCANBus);

    final TalonFXConfiguration config = new TalonFXConfiguration()
      .withMotorOutput(
        new MotorOutputConfigs()
          .withInverted(InvertedValue.CounterClockwise_Positive)
          .withNeutralMode(NeutralModeValue.Brake)
      )
      .withCurrentLimits(
        new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(30))
                    .withSupplyCurrentLimitEnable(true)
      );
    m_motor.getConfigurator().apply(config);
    SmartDashboard.putData(this);
  }

  public void set(Speed speed) {
    m_motor.setControl(
      voltageRequest.withOutput(speed.voltage())
    );
  }

  public Command feedCommand() {
    return startEnd(() -> set(Speed.FEED), () -> set(Speed.STOP));
  }

  private GenericEntry currentCommandEntry = ShuffleboardConstants.kFloorTab.add("Floor/Current Command", 0.0)
      .withWidget(BuiltInWidgets.kCommand)
      .withSize(2, 1)
      .withPosition(0, 0)
      .getEntry();
  private GenericEntry rpmEntry = ShuffleboardConstants.kFloorTab.add("Floor/RPM", 0.0)
      .withWidget(BuiltInWidgets.kTextView)
      .withSize(2, 1)
      .withPosition(2, 0)
      .getEntry();
  private GenericEntry statorCurrentEntry = ShuffleboardConstants.kFloorTab.add("Floor/Stator Current", 0.0)
      .withWidget(BuiltInWidgets.kTextView)
      .withSize(2, 1)
      .withPosition(3, 0)
      .getEntry();
  private GenericEntry supplyCurrentEntry = ShuffleboardConstants.kFloorTab.add("Floor/Supply Current", 0.0)
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
