package frc.robot.commands;

import static frc.robot.generated.ChoreoTraj.LeftStartlineToBallzone;
import static frc.robot.generated.ChoreoTraj.LeftBallzoneToRightBallzone;
import static frc.robot.generated.ChoreoTraj.RightBallzoneToShooting;

import static frc.robot.generated.ChoreoTraj.RightStartlineToBallzone;
import static frc.robot.generated.ChoreoTraj.RightBallzoneToLeftBallzone;
import static frc.robot.generated.ChoreoTraj.RightBallzoneToRight;

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
        autoChooser.addRoutine("Left -> Ballpittt -> Right Shoot", this::L_Bi_Rs);
        autoChooser.addRoutine("Right start -> Ballpit intkake -> Right shoot", this::R_Bi_Rs);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    /*
     * Naming conventions
     * 
     * X-Y-Z
     * 
     * X = Starting position
     * L = Left
     * R = Right
     * C = Center
     * 
     * Y = Primary moving location
     * B = Ball Zone
     * S = Shooting Zone
     * O = Outpost
     * 
     * Z = Secondary moving location
     * R = Right
     * L = Left
     * S = Shooting Zone
     * O = Outpost
     * 
     * LOWER CASE LETTER INDICITAES SUBSYSTEM
     * i = intake
     * s = shooter
     * c = climber
     * 
     */

    // Full autonomous

    private AutoRoutine L_Bi_Rs() {

        boolean AUTO_VERIFIED = false;
        if (!AUTO_VERIFIED) {
            System.out.println("WARNING: Auto routine not verified!");
        }

        final AutoRoutine routine = autoFactory.newRoutine("Forward Auto");
        final AutoTrajectory startToBallzone = LeftStartlineToBallzone.asAutoTraj(routine);
        final AutoTrajectory ballzoneToBallzone = LeftBallzoneToRightBallzone.asAutoTraj(routine);
        final AutoTrajectory ballzoneToShooting = RightBallzoneToShooting.asAutoTraj(routine);

        routine.active().onTrue(
                Commands.sequence(
                        startToBallzone.resetOdometry(),
                        startToBallzone.cmd()));

        routine.observe(hanger::isHomed).onTrue(
                Commands.sequence(
                        Commands.waitSeconds(0.5),
                        intake.runOnce(() -> intake.set(Intake.Position.INTAKE))));

        startToBallzone.doneDelayed(1).onTrue(ballzoneToBallzone.cmd());

        ballzoneToBallzone.atTimeBeforeEnd(1).onTrue(intake.intakeCommand());
        ballzoneToBallzone.doneDelayed(0.1).onTrue(ballzoneToShooting.cmd());

        ballzoneToShooting.active().whileTrue(limelight.idle());
        ballzoneToShooting.atTime(0.5).onTrue(
                Commands.parallel(
                        shooter.spinUpCommand(2600),
                        hood.positionCommand(0.32)));
        ballzoneToShooting.done().onTrue(
                Commands.sequence(
                        m_subsystemCommands.aimAndShoot().withTimeout(5)));

        return routine;

    }

    private AutoRoutine R_Bi_Rs() {

        boolean AUTO_VERIFIED = false;
        if (!AUTO_VERIFIED) {
            System.out.println("WARNING: Auto routine not verified!");
        }

        final AutoRoutine routine = autoFactory.newRoutine("Right start -> Ballpit intkake -> Right shoot");
        final AutoTrajectory rightStartlineToBallzone = RightStartlineToBallzone.asAutoTraj(routine);
        final AutoTrajectory ballzoneToBallzone = RightBallzoneToLeftBallzone.asAutoTraj(routine);
        final AutoTrajectory rightBallzoneToRight = RightBallzoneToRight.asAutoTraj(routine);

        routine.active().onTrue(
                Commands.sequence(
                        rightStartlineToBallzone.resetOdometry(),
                        rightStartlineToBallzone.cmd()));

        routine.observe(hanger::isHomed).onTrue(
                Commands.sequence(
                        Commands.waitSeconds(0.5),
                        intake.runOnce(() -> intake.set(Intake.Position.INTAKE))));

        rightStartlineToBallzone.doneDelayed(1).onTrue(ballzoneToBallzone.cmd());

        ballzoneToBallzone.atTimeBeforeEnd(1).onTrue(intake.intakeCommand());
        ballzoneToBallzone.doneDelayed(0.1).onTrue(rightBallzoneToRight.cmd());

        rightBallzoneToRight.active().whileTrue(limelight.idle());
        rightBallzoneToRight.atTime(0.5).onTrue(
                Commands.parallel(
                        shooter.spinUpCommand(2600),
                        hood.positionCommand(0.32)));
        rightBallzoneToRight.done().onTrue(
                Commands.sequence(
                        m_subsystemCommands.aimAndShoot().withTimeout(5)));

        return routine;
    }
}
