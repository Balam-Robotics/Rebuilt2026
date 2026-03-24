// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.KrakenX60;
import frc.robot.Constants.Ports;
import frc.robot.Constants.ShuffleboardConstants;

public class Intake extends SubsystemBase {
  public enum Speed {
    STOP(0),
    INTAKE(0.8);

    private final double percentOutput;

    private Speed(double percentOutput) {
      this.percentOutput = percentOutput;
    }

    public Voltage voltage() {
      return Volts.of(percentOutput * 12.0);
    }
  }

  public enum Position {
    HOMED(110),
    STOWED(100),
    INTAKE(-4),
    AGITATE(20);

    private final double degrees;

    private Position(double degrees) {
      this.degrees = degrees;
    }

    public Angle angle() {
      return Degrees.of(degrees);
    }
  }

  private static final double kPivotReduction = 50.0;
  private static final AngularVelocity kMaxPivotSpeed = KrakenX60.kFreeSpeed.div(kPivotReduction);
  private static final Angle kPositionTolerance = Degrees.of(5);

  private final TalonFX m_pivotMotor, m_rollerMotor;
  private final VoltageOut pivotVoltageRequest = new VoltageOut(0);
  private final MotionMagicVoltage pivotMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
  private final VoltageOut rollerVoltageRequest = new VoltageOut(0);

  private boolean isHomed = false;

  public Intake() {
    m_pivotMotor = new TalonFX(Ports.kIntakePivot, Ports.kRoboRioCANBus);
    m_rollerMotor = new TalonFX(Ports.kIntakeRollers, Ports.kRoboRioCANBus);
    configurePivotMotor();
    configureRollerMotor();
    SmartDashboard.putData(this);
  }

  private void configurePivotMotor() {
    final TalonFXConfiguration config = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(120))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(70))
                .withSupplyCurrentLimitEnable(true))
        .withFeedback(
            new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                .withSensorToMechanismRatio(kPivotReduction))
        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(kMaxPivotSpeed)
                .withMotionMagicAcceleration(kMaxPivotSpeed.per(Second)))
        .withSlot0(
            new Slot0Configs()
                .withKP(300)
                .withKI(0)
                .withKD(0)
                .withKV(12.0 / kMaxPivotSpeed.in(RotationsPerSecond)) // 12 volts when requesting max RPS
        );
    m_pivotMotor.getConfigurator().apply(config);
  }

  private void configureRollerMotor() {
    final TalonFXConfiguration config = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(120))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(70))
                .withSupplyCurrentLimitEnable(true));
    m_rollerMotor.getConfigurator().apply(config);
  }

  private boolean isPositionWithinTolerance() {
    final Angle currentPosition = m_pivotMotor.getPosition().getValue();
    final Angle targetPosition = pivotMotionMagicRequest.getPositionMeasure();
    return currentPosition.isNear(targetPosition, kPositionTolerance);
  }

  private void setPivotPercentOutput(double percentOutput) {
    m_pivotMotor.setControl(
        pivotVoltageRequest.withOutput(Volts.of(percentOutput * 12.0)));
  }

  // pivot
  public void set(Position position) {
    m_pivotMotor.setControl(
        pivotMotionMagicRequest.withPosition(position.angle()));
  }

  // rollers
  public void set(Speed speed) {
    m_rollerMotor.setControl(
        rollerVoltageRequest.withOutput(speed.voltage()));
  }

  public Command intakeCommand() {
    return startEnd(
        () -> {
          set(Position.INTAKE);
          set(Speed.INTAKE);
        },
        () -> set(Speed.STOP));
  }

  public Command agitateCommand() {
    return runOnce(() -> set(Speed.INTAKE))
        .andThen(
            Commands.sequence(
                runOnce(() -> set(Position.AGITATE)),
                Commands.waitUntil(this::isPositionWithinTolerance),
                runOnce(() -> set(Position.INTAKE)),
                Commands.waitUntil(this::isPositionWithinTolerance))
                .repeatedly())
        .handleInterrupt(() -> {
          set(Position.INTAKE);
          set(Speed.STOP);
        }); 
  }

  public Command homingCommand() {
    return Commands.sequence(
        runOnce(() -> setPivotPercentOutput(0.1)),
        Commands.waitUntil(() -> m_pivotMotor.getSupplyCurrent().getValue().in(Amps) > 6),
        runOnce(() -> {
          m_pivotMotor.setPosition(Position.HOMED.angle());
          isHomed = true;
          set(Position.STOWED);
        }))
        .unless(() -> isHomed)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  private GenericEntry currentCommandEntry = ShuffleboardConstants.kIntakeTab.add("Intake Current Command", 0.0)
  .withWidget(BuiltInWidgets.kCommand)
  .withSize(2, 1)
  .withPosition(0, 0)
  .getEntry();
  private GenericEntry angleEntry = ShuffleboardConstants.kIntakeTab.add("Angle (degrees)", 0.0)
  .withWidget(BuiltInWidgets.kNumberBar)
  .withSize(2, 1)
  .withPosition(2, 0)
  .getEntry();
  private GenericEntry rpmEntry = ShuffleboardConstants.kIntakeTab.add("Roller RPM", 0.0)
  .withWidget(BuiltInWidgets.kNumberBar)
  .withSize(2, 1)
  .withPosition(4, 0)
  .getEntry();
  private GenericEntry pivotCurrentEntry = ShuffleboardConstants.kIntakeTab.add("Pivot Supply Current", 0.0)
  .withWidget(BuiltInWidgets.kNumberBar)
  .withSize(2, 1)
  .withPosition(6, 0)
  .getEntry();
  private GenericEntry rollerCurrentEntry = ShuffleboardConstants.kIntakeTab.add("Roller Supply Current", 0.0)
  .withWidget(BuiltInWidgets.kNumberBar)
  .withSize(2, 1)
  .withPosition(8, 0)
  .getEntry();

  @Override
  public void periodic() {
    currentCommandEntry.setString(getCurrentCommand() != null ? getCurrentCommand().getName() : "null");
    angleEntry.setDouble(m_pivotMotor.getPosition().getValue().in(Degrees));
    rpmEntry.setDouble(m_rollerMotor.getVelocity().getValue().in(RPM));
    pivotCurrentEntry.setDouble(m_pivotMotor.getSupplyCurrent().getValue().in(Amps));
    rollerCurrentEntry.setDouble(m_rollerMotor.getSupplyCurrent().getValue().in(Amps));
  }
}
