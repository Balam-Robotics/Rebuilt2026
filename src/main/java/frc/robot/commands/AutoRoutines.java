package frc.robot.commands;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.Swerve;

public class AutoRoutines {
    
private final AutoFactory autoFactory;
private final AutoChooser autoChooser;

    public AutoRoutines(Swerve swerve) {
        this.autoFactory = swerve.createAutoFactory();
        this.autoChooser = new AutoChooser();
    }

    public void configure() {
        //autoChooser.addRoutine("Demo Path", this::demoPathAuto);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    /*
    public AutoRoutine demoPathAuto() {
        AutoRoutine routine = autoFactory.newRoutine("DemoAuto");
        AutoTrajectory path = ChoreoTraj.NewPath.asAutoTraj(routine);
        routine.active().onTrue(
            Commands.sequence(
                path.resetOdometry(),
                path.cmd()
            )
        );
        
        return routine;
    }
    */
    public Command getSelectedCommand() {
        return autoChooser.selectedCommandScheduler();
    }
}
