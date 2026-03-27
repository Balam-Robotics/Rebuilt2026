// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.util.Gametime;

public class Robot extends TimedRobot {
    @SuppressWarnings("unused")
    private final RobotContainer m_robotContainer;
    private final Gametime gametime = new Gametime();

    public Robot() {
        m_robotContainer = new RobotContainer();
        SmartDashboard.putData(CommandScheduler.getInstance());
        RobotController.setBrownoutVoltage(Volts.of(6.1));

        gametime.addListener(new Gametime.GameStateListener() {
            @Override
            public void onFieldStateChanged(Gametime.FieldState newState, Gametime.FieldState oldState) {
                System.out.println("Field cambió a: " + newState);
            }

            @Override
            public void onChangeNotification(double timeUntilChange) {
                System.out.println("⚠️ CAMBIO en " + timeUntilChange + " segundos");
            }

            @Override
            public void onPhaseChanged(Gametime.GamePhase newPhase, Gametime.GamePhase oldPhase) {
                System.out.println("Fase: " + gametime.getPhaseDisplayName());
            }
        });

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        gametime.periodic();
    }
}
