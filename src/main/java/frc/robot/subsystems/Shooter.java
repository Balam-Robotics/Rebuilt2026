// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;

import java.util.List;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.KrakenX60;
import frc.robot.Constants.Ports;
import frc.robot.Constants.ShuffleboardConstants;

public class Shooter extends SubsystemBase {
  private static final AngularVelocity kVelocityTolerance = RPM.of(100);

  private final TalonFX m_leftMotor, m_middleMotor, m_rightMotor;
  private final List<TalonFX> m_motors;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private double dashboardTargetRPM = 3600.0;

  public Shooter() {
    m_leftMotor = new TalonFX(Ports.kShooterLeft, Ports.kRoboRioCANBus);
    m_middleMotor = new TalonFX(Ports.kShooterMiddle, Ports.kRoboRioCANBus);
    m_rightMotor = new TalonFX(Ports.kShooterRight, Ports.kRoboRioCANBus);
    m_motors = List.of(m_leftMotor, m_middleMotor, m_rightMotor);

    configureMotor(m_leftMotor, InvertedValue.Clockwise_Positive);
    configureMotor(m_middleMotor, InvertedValue.CounterClockwise_Positive);
    configureMotor(m_rightMotor, InvertedValue.CounterClockwise_Positive);

    SmartDashboard.putData(this);
  }

    private void configureMotor(TalonFX motor, InvertedValue invertDirection) {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(invertDirection)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withVoltage(
                new VoltageConfigs()
                    .withPeakReverseVoltage(Volts.of(0))
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(70))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(0.5)
                    .withKI(2)
                    .withKD(0)
                    .withKV(12.0 / KrakenX60.kFreeSpeed.in(RotationsPerSecond)) // 12 volts when requesting max RPS
            );
        
        motor.getConfigurator().apply(config);
    }

    public void setRPM(double rpm) {
      for (final TalonFX motor : m_motors) {
        motor.setControl(
          velocityRequest.withVelocity(RPM.of(rpm))
        );
      }
    }

    public void setPercentOutput(double percentOutput) {
      for (final TalonFX motor : m_motors) {
        motor.setControl(
          voltageRequest.withOutput(Volts.of(percentOutput * 12.0))
        );
      }
    }

    public void stop() {
      setPercentOutput(0.0);
    }

    public Command spinUpCommand(double rpm) {
      return runOnce(() -> setRPM(rpm)).andThen(Commands.waitUntil(this::isVelocityWithinTolerance));
    }

    public Command dashboardSpinUpCommand() {
      return defer(() -> spinUpCommand(dashboardTargetRPM));
    }

    public Command unstuckCommand() {
      return startEnd(() -> setPercentOutput(-1), () -> setPercentOutput(0));
    }

    public boolean isVelocityWithinTolerance() {
      return m_motors.stream().allMatch(motor -> {
        final boolean isInVelocityMode = motor.getAppliedControl().equals(velocityRequest);
        final AngularVelocity currentVelocity = motor.getVelocity().getValue();
        final AngularVelocity targetVelocity = velocityRequest.getVelocityMeasure();
        return isInVelocityMode && currentVelocity.isNear(targetVelocity, kVelocityTolerance);
      });
    }

    private GenericEntry currentCommandEntry = ShuffleboardConstants.kShooterTab.add("Current Command", 0.0)
    .withWidget(BuiltInWidgets.kCommand)
    .withSize(2, 1)
    .withPosition(0, 0)
    .getEntry();
    private GenericEntry dashboardTargetRPMEntry = ShuffleboardConstants.kShooterTab.add("Dashboard Target RPM", 0.0)
    .withWidget(BuiltInWidgets.kNumberBar)
    .withSize(2, 1)
    .withPosition(2, 0)
    .getEntry();
    private GenericEntry targetRPMEntry = ShuffleboardConstants.kShooterTab.add("Target RPM", 0.0)
    .withWidget(BuiltInWidgets.kNumberBar)
    .withSize(2, 1)
    .withPosition(4, 0)
    .getEntry();
  
    // Left motor information

    private GenericEntry leftMotorVoltageEntry = ShuffleboardConstants.kShooterTab.add("Left Motor Voltage", 0.0)
    .withWidget(BuiltInWidgets.kVoltageView)
    .withSize(2, 1)
    .withPosition(0, 2)
    .getEntry();
    private GenericEntry leftMotorCurrentEntry = ShuffleboardConstants.kShooterTab.add("Left Motor Current", 0.0)
    .withWidget(BuiltInWidgets.kVoltageView)
    .withSize(2, 1)
    .withPosition(2, 2)
    .getEntry();
    private GenericEntry leftMotorAccelerationEntry = ShuffleboardConstants.kShooterTab.add("Left Motor Acceleration", 0.0)
    .withWidget(BuiltInWidgets.kNumberBar)
    .withSize(2, 1)
    .withPosition(4, 2)
    .getEntry();  

    // Middle motor information

    private GenericEntry middleMotorVoltageEntry = ShuffleboardConstants.kShooterTab.add("Middle Motor Voltage", 0.0)
    .withWidget(BuiltInWidgets.kVoltageView)
    .withSize(2, 1)
    .withPosition(0, 2)
    .getEntry();
    private GenericEntry middleMotorCurrentEntry = ShuffleboardConstants.kShooterTab.add("Middle Motor Current", 0.0)
    .withWidget(BuiltInWidgets.kVoltageView)
    .withSize(2, 1)
    .withPosition(2, 2)
    .getEntry();
    private GenericEntry middleMotorAccelerationEntry = ShuffleboardConstants.kShooterTab.add("Middle Motor Acceleration", 0.0)
    .withWidget(BuiltInWidgets.kNumberBar)
    .withSize(2, 1)
    .withPosition(4, 2)
    .getEntry();  

    // Right motor information

    private GenericEntry rightMotorVoltageEntry = ShuffleboardConstants.kShooterTab.add("Right Motor Voltage", 0.0)
    .withWidget(BuiltInWidgets.kVoltageView)
    .withSize(2, 1)
    .withPosition(0, 3)
    .getEntry();
    private GenericEntry rightMotorCurrentEntry = ShuffleboardConstants.kShooterTab.add("Right Motor Current", 0.0)
    .withWidget(BuiltInWidgets.kVoltageView)
    .withSize(2, 1)
    .withPosition(2, 3)
    .getEntry();
    private GenericEntry rightMotorAccelerationEntry = ShuffleboardConstants.kShooterTab.add("Right Motor Acceleration", 0.0)
    .withWidget(BuiltInWidgets.kNumberBar)
    .withSize(2, 1)
    .withPosition(4, 3)
    .getEntry();

    @Override
    public void periodic() {
        currentCommandEntry.setString(getCurrentCommand() != null ? getCurrentCommand().getName() : "null");
        dashboardTargetRPMEntry.setDouble(dashboardTargetRPM);
        targetRPMEntry.setDouble(velocityRequest.getVelocityMeasure().in(RPM));
/* 
        leftMotorVoltageEntry.setDouble(((Voltage) m_leftMotor.getMotorVoltage()).in(Volts));
        leftMotorCurrentEntry.setDouble(((Current) m_leftMotor.getSupplyCurrent().getValue()).in(Amps));
        leftMotorAccelerationEntry.setDouble(((AngularVelocity) m_leftMotor.getAcceleration()).in(RPM));

        middleMotorVoltageEntry.setDouble(((Voltage) m_middleMotor.getMotorVoltage()).in(Volts));
        middleMotorCurrentEntry.setDouble(((Current) m_middleMotor.getSupplyCurrent().getValue()).in(Amps));
        middleMotorAccelerationEntry.setDouble(((AngularVelocity) m_middleMotor.getAcceleration()).in(RPM));

        rightMotorVoltageEntry.setDouble(((Voltage) m_rightMotor.getMotorVoltage()).in(Volts));
        rightMotorCurrentEntry.setDouble(((Current) m_rightMotor.getSupplyCurrent().getValue()).in(Amps));
        rightMotorAccelerationEntry.setDouble(((AngularVelocity) m_rightMotor.getAcceleration()).in(RPM));
        */
    }

    @Override
    public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty("Dashboard RPM",() -> dashboardTargetRPM, value -> dashboardTargetRPM = value);
    }
}
