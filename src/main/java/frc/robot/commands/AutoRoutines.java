package frc.robot.commands;

import static frc.robot.generated.ChoreoTraj.Forward;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class AutoRoutines {
    private final Swerve swerve;
    private final Intake intake;
    private final Floor floor;
    private final Feeder feeder;
    private final Shooter shooter;
    private final Hood hood;
    private final Hanger hanger;
    private final Limelight limelight;

    private final SubsystemCommands m_subsystemCommands;

    private final AutoFactory autoFactory;
    private final AutoChooser autoChooser;

    public AutoRoutines(
            Swerve swerve,
            Intake intake,
            Floor floor,
            Feeder feeder,
            Shooter shooter,
            Hood hood,
            Hanger hanger,
            Limelight limelight) {
        this.swerve = swerve;
        this.intake = intake;
        this.floor = floor;
        this.feeder = feeder;
        this.shooter = shooter;
        this.hood = hood;
        this.hanger = hanger;
        this.limelight = limelight;

        this.m_subsystemCommands = new SubsystemCommands(swerve, intake, floor, feeder, shooter, hood, hanger);

        this.autoFactory = swerve.createAutoFactory();
        this.autoChooser = new AutoChooser();
    }

    public void configure() {
        autoChooser.addRoutine("DemoPath", this::forwardAuto);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    private AutoRoutine forwardAuto() {
        final AutoRoutine routine = autoFactory.newRoutine("Forward Auto");
        final AutoTrajectory forward = Forward.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                forward.resetOdometry(),
                forward.cmd()
            )
        );

        return routine;

    }
}
