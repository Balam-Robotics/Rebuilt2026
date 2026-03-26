package frc.robot.commands;

import static frc.robot.generated.ChoreoTraj.A_RightStartlineToAlliedRightBallzone;
import static frc.robot.generated.ChoreoTraj.RightBallzoneToLeftBallzone;
import static frc.robot.generated.ChoreoTraj.B_AlliedRightBallzoneToAlliedMiddleBallzone;

import static frc.robot.generated.ChoreoTraj.A_LeftStartlineToAlliedBallzone;
import static frc.robot.generated.ChoreoTraj.B_LeftBallzoneToRightBallzone;
import static frc.robot.generated.ChoreoTraj.LeftBallzoneToLeft;

import static frc.robot.generated.ChoreoTraj.CenterStartlineToLeft;
import static frc.robot.generated.ChoreoTraj.Seminuevo;

import static frc.robot.generated.ChoreoTraj.CenterSl_RightABz_Shootingzone;
import static frc.robot.generated.ChoreoTraj.CenterSl_RightABz_Shootingzone_copy1;
import static frc.robot.generated.ChoreoTraj.CenterSl_RightABz_Shootingzone_PrepTeleop;

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

        private final Field2d m_field;

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

                this.m_field = swerve.getField();
        }

        public void configure() {
                // autoChooser.addRoutine("Right start -> Ballpit intkake -> Right shoot",
                // this::R_Bi_Rs);
                autoChooser.addRoutine("Left start -> Ballpit intake -> Left shoot",
                                this::L_Bi_Ls);
                autoChooser.addRoutine("Center-left start -> Ballpit intake -> Left shoot", this::C_Bi_Ls);
                autoChooser.addRoutine("prueba", this::prueba);
                autoChooser.addRoutine("prueba 2", this::prueba_2);
                // autoChooser.addRoutine("cami", this::cami);
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

        private AutoRoutine R_Bi_Rs() {

                boolean AUTO_VERIFIED = false;
                if (!AUTO_VERIFIED) {
                        System.out.println("WARNING: Auto routine not verified!");
                }

                final AutoRoutine routine = autoFactory.newRoutine("Right start -> Ballpit intkake -> Right shoot");
                final AutoTrajectory rightStartlineToBallzone = A_RightStartlineToAlliedRightBallzone
                                .asAutoTraj(routine);
                final AutoTrajectory ballzoneToBallzone = RightBallzoneToLeftBallzone.asAutoTraj(routine);
                final AutoTrajectory rightBallzoneToRight = B_AlliedRightBallzoneToAlliedMiddleBallzone
                                .asAutoTraj(routine);

                routine.active().onTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> {

                                                        m_field.getObject("traj1").setPoses(rightStartlineToBallzone
                                                                        .getRawTrajectory().getPoses());
                                                        m_field.getObject("traj2").setPoses(ballzoneToBallzone
                                                                        .getRawTrajectory().getPoses());
                                                        m_field.getObject("traj3").setPoses(rightBallzoneToRight
                                                                        .getRawTrajectory().getPoses());
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
                final AutoTrajectory leftStartlineToBallzone = A_LeftStartlineToAlliedBallzone.asAutoTraj(routine);
                final AutoTrajectory leftBallzoneToRightBallzone = B_LeftBallzoneToRightBallzone.asAutoTraj(routine);
                final AutoTrajectory leftBallzoneToRight = LeftBallzoneToLeft.asAutoTraj(routine);

                routine.active().onTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> {

                                                        m_field.getObject("traj1").setPoses(leftStartlineToBallzone
                                                                        .getRawTrajectory().getPoses());
                                                        m_field.getObject("traj2").setPoses(leftBallzoneToRightBallzone
                                                                        .getRawTrajectory().getPoses());
                                                        m_field.getObject("traj3").setPoses(leftBallzoneToRight
                                                                        .getRawTrajectory().getPoses());
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

        private AutoRoutine C_Bi_Ls() {

                boolean AUTO_VERIFIED = false;
                if (!AUTO_VERIFIED) {
                        System.out.println("WARNING: Auto routine not verified!");
                }

                final AutoRoutine routine = autoFactory.newRoutine("Center-left start -> Ballpit intake -> Left shoot");
                final AutoTrajectory centerStartlineToLeft = CenterStartlineToLeft.asAutoTraj(routine);
                final AutoTrajectory centerBallpitToLeftShot = Seminuevo.asAutoTraj(routine);

                routine.active().onTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> {

                                                        m_field.getObject("traj1").setPoses(centerStartlineToLeft
                                                                        .getRawTrajectory().getPoses());
                                                        m_field.getObject("traj2").setPoses(centerBallpitToLeftShot
                                                                        .getRawTrajectory().getPoses());
                                                }),
                                                centerStartlineToLeft.resetOdometry(),
                                                centerStartlineToLeft.cmd()));

                routine.observe(hanger::isHomed).onTrue(
                                Commands.sequence(
                                                Commands.waitSeconds(0.5),
                                                intake.runOnce(() -> intake.set(Intake.Position.INTAKE))));

                centerStartlineToLeft.atTime("intake").onTrue(intake.intakeCommand());

                centerStartlineToLeft.doneDelayed(0.1).onTrue(centerBallpitToLeftShot.cmd());

                centerBallpitToLeftShot.atTime(1.0).onTrue(
                                Commands.parallel(
                                                shooter.spinUpCommand(2600),
                                                hood.positionCommand(0.32)));
                centerBallpitToLeftShot.done().onTrue(
                                Commands.sequence(
                                                m_subsystemCommands.aimAndShoot().withTimeout(5)));

                return routine;
        }

        private AutoRoutine prueba() {

                boolean AUTO_VERIFIED = false;
                if (!AUTO_VERIFIED) {
                        System.out.println("WARNING: Auto routine not verified!");
                }

                final AutoRoutine routine = autoFactory.newRoutine("prueba");
                final AutoTrajectory auto = CenterSl_RightABz_Shootingzone_copy1
                                .asAutoTraj(routine);

                routine.active().onTrue(
                                Commands.sequence(
                                                auto.resetOdometry(),
                                                auto.cmd()));

                routine.observe(hanger::isHomed).onTrue(
                                Commands.sequence(
                                                Commands.waitSeconds(0.5),
                                                intake.runOnce(() -> intake.set(Intake.Position.INTAKE))));

                auto.atTime("INTAKE").onTrue(intake.intakeCommand());

                auto.atTime("PrepShooter").onTrue(
                                Commands.parallel(
                                                shooter.spinUpCommand(2600),
                                                hood.positionCommand(0.32)));
                auto.done().onTrue(
                                Commands.sequence(
                                                m_subsystemCommands.aimAndShoot().withTimeout(5)));

                return routine;
        }

private AutoRoutine prueba_2() {

                boolean AUTO_VERIFIED = false;
                if (!AUTO_VERIFIED) {
                        System.out.println("WARNING: Auto routine not verified!");
                }

                final AutoRoutine routine = autoFactory.newRoutine("prueba");
                final AutoTrajectory auto = CenterSl_RightABz_Shootingzone_PrepTeleop
                                .asAutoTraj(routine);

                routine.active().onTrue(
                                Commands.sequence(
                                                auto.resetOdometry(),
                                                auto.cmd()));

                routine.observe(hanger::isHomed).onTrue(
                                Commands.sequence(
                                                Commands.waitSeconds(0.5),
                                                intake.runOnce(() -> intake.set(Intake.Position.INTAKE))));

                auto.atTime("INTAKE").onTrue(intake.intakeCommand());

                auto.atTime("PrepShooter").onTrue(
                                Commands.parallel(
                                                shooter.spinUpCommand(2600),
                                                hood.positionCommand(0.32)));
                auto.atTime("SHOOT").onTrue(
                                Commands.sequence(
                                                m_subsystemCommands.aimAndShoot().withTimeout(5),
                                                Commands.waitSeconds(5)));

                return routine;
        }


        /*
         * 
         * private AutoRoutine C_Bi_Ss() {
         * 
         * boolean AUTO_VERIFIED = false;
         * if (!AUTO_VERIFIED) {
         * System.out.println("WARNING: Auto routine not verified!");
         * }
         * 
         * final AutoRoutine routine =
         * AutoFatory.newroutine("Center-right start -> Ballpit intake -> Right shoot");
         * final AutoTrajectory CenterSl_RightABz_Shootingzone =
         * CenterSl_RightABz_Shootingzone.asAutoTraj(routine);
         * 
         * routine.active().onTrue(
         * Commands.sequence(
         * Commands.runOnce(() -> {
         * 
         * m_field.getObject("traj1").setPoses(CenterSl_RightABz_Shootingzone.
         * getRawTrajectory().getPoses());
         * 
         * 
         * }),
         * CenterSl_RightABz_Shootingzone.resetOdometry(),
         * CenterSl_RightABz_Shootingzone.cmd));
         * 
         * routine.observe(hanger::isHomed).onTrue(
         * Commands.sequence(
         * Commands.waitSeconds(0.5),
         * intake.runOnce(() -> intake.set(Intake.Position.INTAKE))));
         * 
         * CenterSl_RightABz_Shootingzone.atTime("Intake").onTrue(intake.intakeCommand()
         * );
         * CenterSl_RightABz_Shootingzone.atTime("PrepShooter").onTrue(());
         * 
         * 
         * };
         * return routine;
         */
}
