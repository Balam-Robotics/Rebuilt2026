package frc.robot.commands;

import static frc.robot.generated.ChoreoTraj.RightStartlineToBallzone;
import static frc.robot.generated.ChoreoTraj.cami;
import static frc.robot.generated.ChoreoTraj.RightBallzoneToLeftBallzone;
import static frc.robot.generated.ChoreoTraj.RightBallzoneToRight;

import static frc.robot.generated.ChoreoTraj.RightShootToMiddleBallzone;
import static frc.robot.generated.ChoreoTraj.MiddleRightBallzoneToMiddleLeftBallzone;
import static frc.robot.generated.ChoreoTraj.MiddleBallzoneToRightShoot;

import static frc.robot.generated.ChoreoTraj.LeftStartlineToBallzone;
import static frc.robot.generated.ChoreoTraj.LeftBallzoneToRightBallzone;
import static frc.robot.generated.ChoreoTraj.LeftBallzoneToLeft;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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

    private final Field2d m_field = new Field2d();

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
        autoChooser.addRoutine("Right start -> Ballpit intkake -> Right shoot", this::R_Bi_Rs);
        autoChooser.addRoutine("Left start -> Ballpit intake -> Left shoot", this::L_Bi_Ls);
        autoChooser.addRoutine("Right start -> Ballpit intkake -> Right shoot x2", this::R_Bi_Rs_Bi_Rs);
        autoChooser.addRoutine("cami", this::cami);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
        SmartDashboard.putData("Traj Field", m_field);
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
                        Commands.runOnce(() -> {

                            m_field.getObject("traj1").setPoses(rightStartlineToBallzone.getRawTrajectory().getPoses());
                            m_field.getObject("traj2").setPoses(ballzoneToBallzone.getRawTrajectory().getPoses());
                            m_field.getObject("traj3").setPoses(rightBallzoneToRight.getRawTrajectory().getPoses());
                        }),
                        rightStartlineToBallzone.resetOdometry(),
                        rightStartlineToBallzone.cmd()));

        routine.observe(hanger::isHomed).onTrue(
                Commands.sequence(
                        Commands.waitSeconds(0.5),
                        intake.runOnce(() -> intake.set(Intake.Position.INTAKE))));

        rightStartlineToBallzone.doneDelayed(0.125).onTrue(ballzoneToBallzone.cmd());

        ballzoneToBallzone.atTime(0).onTrue(intake.intakeCommand());
        ballzoneToBallzone.doneDelayed(0.1).onTrue(rightBallzoneToRight.cmd());

        // rightBallzoneToRight.active().whileTrue(limelight.idle()); -> shit is making
        // limelight not work?
        rightBallzoneToRight.atTime(0.5).onTrue(
                Commands.parallel(
                        shooter.spinUpCommand(2600),
                        hood.positionCommand(0.32)));
        rightBallzoneToRight.done().onTrue(
                Commands.sequence(
                        m_subsystemCommands.aimAndShoot().withTimeout(5)));

        return routine;
    }

    private AutoRoutine L_Bi_Ls() {

        boolean AUTO_VERIFIED = false;
        if (!AUTO_VERIFIED) {
            System.out.println("WARNING: Auto routine not verified!");
        }

        final AutoRoutine routine = autoFactory.newRoutine("Left start -> Ballpit intake -> Left shoot");
        final AutoTrajectory leftStartlineToBallzone = LeftStartlineToBallzone.asAutoTraj(routine);
        final AutoTrajectory leftBallzoneToRightBallzone = LeftBallzoneToRightBallzone.asAutoTraj(routine);
        final AutoTrajectory leftBallzoneToRight = LeftBallzoneToLeft.asAutoTraj(routine);

        routine.active().onTrue(
                Commands.sequence(
                        Commands.runOnce(() -> {

                            m_field.getObject("traj1").setPoses(leftStartlineToBallzone.getRawTrajectory().getPoses());
                            m_field.getObject("traj2").setPoses(leftBallzoneToRightBallzone.getRawTrajectory().getPoses());
                            m_field.getObject("traj3").setPoses(leftBallzoneToRight.getRawTrajectory().getPoses());
                        }),
                        leftStartlineToBallzone.resetOdometry(),
                        leftStartlineToBallzone.cmd()));

        routine.observe(hanger::isHomed).onTrue(
                Commands.sequence(
                        Commands.waitSeconds(0.5),
                        intake.runOnce(() -> intake.set(Intake.Position.INTAKE))));

        leftStartlineToBallzone.doneDelayed(0).onTrue(leftBallzoneToRightBallzone.cmd());

        leftBallzoneToRightBallzone.atTime(0).onTrue(intake.intakeCommand());
        leftBallzoneToRightBallzone.doneDelayed(0.1).onTrue(leftBallzoneToRight.cmd());

        leftBallzoneToRight.atTime(0.5).onTrue(
                Commands.parallel(
                        shooter.spinUpCommand(2600),
                        hood.positionCommand(0.32)));
        leftBallzoneToRight.done().onTrue(
                Commands.sequence(
                        m_subsystemCommands.aimAndShoot().withTimeout(5)));

        return routine;
    }

    private AutoRoutine R_Bi_Rs_Bi_Rs() {

        boolean AUTO_VERIFIED = false;
        if (!AUTO_VERIFIED) {
            System.out.println("WARNING: Auto routine not verified!");
        }

        final AutoRoutine routine = autoFactory
                .newRoutine("Right start -> Ballpit intkake -> Right shoot -> Middle ballpit -> Right Shoot");
        final AutoTrajectory rightStartlineToBallzone = RightStartlineToBallzone.asAutoTraj(routine);
        final AutoTrajectory ballzoneToBallzone = RightBallzoneToLeftBallzone.asAutoTraj(routine);
        final AutoTrajectory rightBallzoneToRight = RightBallzoneToRight.asAutoTraj(routine);

        final AutoTrajectory rightShootToMiddleBallzone = RightShootToMiddleBallzone.asAutoTraj(routine);
        final AutoTrajectory middleRightBallzoneToMiddleLeftBallzone = MiddleRightBallzoneToMiddleLeftBallzone
                .asAutoTraj(routine);
        final AutoTrajectory middleBallzoneToRightShoot = MiddleBallzoneToRightShoot.asAutoTraj(routine);

        routine.active().onTrue(
                Commands.sequence(
                        Commands.runOnce(() -> {

                            m_field.getObject("traj1").setPoses(rightStartlineToBallzone.getRawTrajectory().getPoses());
                            m_field.getObject("traj2").setPoses(ballzoneToBallzone.getRawTrajectory().getPoses());
                            m_field.getObject("traj3").setPoses(rightBallzoneToRight.getRawTrajectory().getPoses());
                        }),
                        rightStartlineToBallzone.resetOdometry(),
                        rightStartlineToBallzone.cmd()));

        routine.observe(hanger::isHomed).onTrue(
                Commands.sequence(
                        Commands.waitSeconds(0.5),
                        intake.runOnce(() -> intake.set(Intake.Position.INTAKE))));

        // First cycle

        rightStartlineToBallzone.doneDelayed(0.125).onTrue(ballzoneToBallzone.cmd());

        ballzoneToBallzone.atTime(0).onTrue(intake.intakeCommand());
        ballzoneToBallzone.doneDelayed(0.1).onTrue(rightBallzoneToRight.cmd());

        // rightBallzoneToRight.active().whileTrue(limelight.idle()); -> shit is making
        // limelight not work?
        rightBallzoneToRight.atTime(0.5).onTrue(
                Commands.parallel(
                        shooter.spinUpCommand(2600),
                        hood.positionCommand(0.32)));
        rightBallzoneToRight.done().onTrue(
                Commands.sequence(
                        m_subsystemCommands.aimAndShoot().withTimeout(5)));

        // Second cycle

        rightBallzoneToRight.doneDelayed(5).onTrue(rightShootToMiddleBallzone.cmd());

        rightShootToMiddleBallzone.doneDelayed(0.125).onTrue(middleRightBallzoneToMiddleLeftBallzone.cmd());

        middleRightBallzoneToMiddleLeftBallzone.atTime(0).onTrue(intake.intakeCommand());
        middleRightBallzoneToMiddleLeftBallzone.doneDelayed(0.1).onTrue(middleBallzoneToRightShoot.cmd());

        middleBallzoneToRightShoot.atTime(0.5).onTrue(
                Commands.parallel(
                        shooter.spinUpCommand(2600),
                        hood.positionCommand(0.32)));
        middleBallzoneToRightShoot.done().onTrue(
                Commands.sequence(
                        m_subsystemCommands.aimAndShoot().withTimeout(5)));

        return routine;
    }



    private AutoRoutine cami() {

        boolean AUTO_VERIFIED = false;
        if (!AUTO_VERIFIED) {
            System.out.println("WARNING: Auto routine not verified!");
        }

        final AutoRoutine routine = autoFactory
                .newRoutine("Right start -> Ballpit intkake -> Right shoot -> Middle ballpit -> Right Shoot");
        final AutoTrajectory camiTraj = cami.asAutoTraj(routine);

        routine.active().onTrue(
                Commands.sequence(
                        Commands.runOnce(() -> {

                            m_field.getObject("traj1").setPoses(camiTraj.getRawTrajectory().getPoses());
                            m_field.getObject("traj2").setPoses(camiTraj.getRawTrajectory().getPoses());
                            m_field.getObject("traj3").setPoses(camiTraj.getRawTrajectory().getPoses());
                        }),
                        camiTraj.resetOdometry(),
                        camiTraj.cmd()));

        routine.observe(hanger::isHomed).onTrue(
                Commands.sequence(
                        Commands.waitSeconds(0.5),
                        intake.runOnce(() -> intake.set(Intake.Position.INTAKE))));

        camiTraj.atTime(3.5).onTrue(intake.intakeCommand());

        // rightBallzoneToRight.active().whileTrue(limelight.idle()); -> shit is making
        // limelight not work?
        camiTraj.atTime(7.5).onTrue(
                Commands.parallel(
                        shooter.spinUpCommand(2600),
                        hood.positionCommand(0.32)));
        camiTraj.done().onTrue(
                Commands.sequence(
                        m_subsystemCommands.aimAndShoot().withTimeout(5)));


        return routine;
    }
}
