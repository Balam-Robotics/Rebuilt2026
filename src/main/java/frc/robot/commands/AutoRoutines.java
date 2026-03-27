package frc.robot.commands;

// Phase A - Starting trajectories
import static frc.robot.generated.ChoreoTraj.A_CenterStartlineToLeftBallzone;
import static frc.robot.generated.ChoreoTraj.A_CenterStartlineToLeftRivalBallzone;
import static frc.robot.generated.ChoreoTraj.A_CenterStartlineToRightBallzone;
import static frc.robot.generated.ChoreoTraj.A_CenterStartlineToRivalRightBallzone;
import static frc.robot.generated.ChoreoTraj.A_LeftStartlineToAlliedBallzone;
import static frc.robot.generated.ChoreoTraj.A_RightStartlineToAlliedRightBallzone;

// Phase B - Ball collection trajectories
import static frc.robot.generated.ChoreoTraj.B_RightBallzoneToLeftBallzone;
import static frc.robot.generated.ChoreoTraj.B_RightBallzoneToMiddleBallzone;
import static frc.robot.generated.ChoreoTraj.B_LeftBallzoneToLeftBallzone;
import static frc.robot.generated.ChoreoTraj.B_LeftBallzoneToMiddleBallzone;

// Phase C - Shooting trajectories
import static frc.robot.generated.ChoreoTraj.C_LeftBallzoneToRightShoot; // C_LeftBallzoneToRightShoot
import static frc.robot.generated.ChoreoTraj.C_RightBallzoneToLeftShoot; // C_RightBallzoneToLeftShoot

import static frc.robot.generated.ChoreoTraj.AutoToOutpost;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.ShuffleboardConstants;
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
        private final GenericEntry AUTO_DELAY = ShuffleboardConstants.kAutonomousTab.add("Auto Delay", 0.0).getEntry();

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
                // === RIGHT SIDE ROUTINES ===
                autoChooser.addRoutine("RIGHT: Ballpit Mid → RIGHT Shoot", this::R_Bm_Rs);
                autoChooser.addRoutine("RIGHT: Ballpit Long → RIGHT Shoot", this::R_Bl_Rs);

                // === LEFT SIDE ROUTINES ===
                autoChooser.addRoutine("LEFT: Ballpit Mid → LEFT Shoot", this::L_Bm_Ls);
                autoChooser.addRoutine("LEFT: Ballpit Long → LEFT Shoot", this::L_Bl_Ls);

                autoChooser.addRoutine("CENTER: Shoot", this::C_O);   
                autoChooser.addRoutine("LEFT: Ballpit Long -> Left Shoot", this::_CL_Bl_Ls);          

                // === CENTER-LEFT ROUTINES ===
                autoChooser.addRoutine("CENTER-LEFT: Ballpit Mid → LEFT Shoot",
                this::CL_Bm_Ls);
                // autoChooser.addRoutine("CENTER-LEFT: Ballpit Long → LEFT Shoot",
                // this::CL_Bl_Ls);
                // autoChooser.addRoutine("CENTER-LEFT: Ballpit Long → RIGHT Shoot",
                // this::CL_Bl_Rs);

                // === CENTER-RIGHT ROUTINES ===
                autoChooser.addRoutine("CENTER-RIGHT: Ballpit Mid → RIGHT Shoot",
                this::CR_Bm_Rs);
                // autoChooser.addRoutine("CENTER-RIGHT: Ballpit Long → RIGHT Shoot",
                // this::CR_Bl_Rs);
                // autoChooser.addRoutine("CENTER-RIGHT: Ballpit Mid → LEFT Shoot",
                // this::CR_Bm_Ls);
                // autoChooser.addRoutine("CENTER-RIGHT: Ballpit Long → LEFT Shoot",
                // this::CR_Bl_Ls);

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
         * C = Center (CL = Center-Left, CR = Center-Right)
         * 
         * Y = Primary ball collection location
         * B = Ball Zone (with m = mid distance, l = long distance)
         * 
         * Z = Shooting location
         * s = shoot (with R = right, L = left)
         * 
         * PATTERN: [StartPos]_[BallZoneVariant]_[ShootPos]
         * Example: R_Bm_Rs = Right start, Ball zone mid distance, Right shoot
         */

        // ========== HELPER METHOD ==========
        private void setupAutoRoutine(AutoRoutine routine, AutoTrajectory autoA, AutoTrajectory autoB,
                        AutoTrajectory autoC,
                        double shooterRPM, double hoodPosition) {

                // Wait the duration of the auto delay variabe
                double wait = AUTO_DELAY.getDouble(0.0);

                routine.active().onTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> {
                                                        m_field.getObject("traj1")
                                                                        .setPoses(autoA.getRawTrajectory().getPoses());
                                                        m_field.getObject("traj2")
                                                                        .setPoses(autoB.getRawTrajectory().getPoses());
                                                        m_field.getObject("traj3")
                                                                        .setPoses(autoC.getRawTrajectory().getPoses());
                                                }),
                                                Commands.runOnce(() -> new WaitCommand(wait)),
                                                autoA.resetOdometry(),
                                                autoA.cmd()));

                routine.observe(hanger::isHomed).onTrue(
                                Commands.sequence(
                                                Commands.waitSeconds(1),
                                                intake.runOnce(() -> intake.set(Intake.Position.INTAKE))));

                autoA.doneDelayed(0.125).onTrue(autoB.cmd());

                autoB.atTime(0).onTrue(intake.intakeCommand());
                autoB.doneDelayed(0.1).onTrue(autoC.cmd());

                autoC.atTime(0).onTrue(
                                Commands.parallel(
                                                shooter.spinUpCommand(shooterRPM),
                                                hood.positionCommand(hoodPosition)));
                autoC.done().onTrue(
                                Commands.sequence(
                                                m_subsystemCommands.aimAndShoot().withTimeout(5)));
        }

        // ========== SIMPLE ROUTINES ==============

        private AutoRoutine C_O() {

                final AutoRoutine routine = autoFactory.newRoutine("RIGHT: Ballpit Mid → RIGHT Shoot");
                final AutoTrajectory auto = AutoToOutpost.asAutoTraj(routine);

                routine.active().onTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> {
                                                        m_field.getObject("traj1")
                                                                        .setPoses(auto.getRawTrajectory().getPoses());
                                                        m_field.getObject("traj2")
                                                                        .setPoses(auto.getRawTrajectory().getPoses());
                                                        m_field.getObject("traj3")
                                                                        .setPoses(auto.getRawTrajectory().getPoses());
                                                }),
                                                auto.resetOdometry(),
                                                auto.cmd()));

                routine.observe(hanger::isHomed).onTrue(
                                Commands.sequence(
                                                Commands.waitSeconds(1),
                                                intake.runOnce(() -> intake.set(Intake.Position.INTAKE))));


                auto.doneDelayed(0.1).onTrue(auto.cmd());
                auto.done().onTrue(
                                Commands.sequence(
                                                m_subsystemCommands.aimAndShoot().withTimeout(5)));

                return routine;

        }

        // ========== RIGHT SIDE ROUTINES ==========

        /**
         * RIGHT: Ballpit Mid Distance → RIGHT Shoot
         * Starts from right, collects from middle ball zone, shoots from right
         */
        private AutoRoutine R_Bm_Rs() {
                boolean AUTO_VERIFIED = true;
                if (!AUTO_VERIFIED) {
                        System.out.println("WARNING: R_Bm_Rs routine not verified!");
                }

                final AutoRoutine routine = autoFactory.newRoutine("RIGHT: Ballpit Mid → RIGHT Shoot");
                final AutoTrajectory autoA = A_RightStartlineToAlliedRightBallzone.asAutoTraj(routine);
                final AutoTrajectory autoB = B_RightBallzoneToMiddleBallzone.asAutoTraj(routine);
                final AutoTrajectory autoC = C_RightBallzoneToLeftShoot.asAutoTraj(routine);

                setupAutoRoutine(routine, autoA, autoB, autoC, 2400, 0.32);
                return routine;
        }

        /**
         * RIGHT: Ballpit Long Distance → RIGHT Shoot
         * Starts from right, collects from far ball zone, shoots from right
         */
        private AutoRoutine R_Bl_Rs() {
                boolean AUTO_VERIFIED = true;
                if (!AUTO_VERIFIED) {
                        System.out.println("WARNING: R_Bl_Rs routine not verified!");
                }

                final AutoRoutine routine = autoFactory.newRoutine("RIGHT: Ballpit Long → RIGHT Shoot");
                final AutoTrajectory autoA = A_RightStartlineToAlliedRightBallzone.asAutoTraj(routine);
                final AutoTrajectory autoB = B_RightBallzoneToLeftBallzone.asAutoTraj(routine);
                final AutoTrajectory autoC = C_LeftBallzoneToRightShoot.asAutoTraj(routine);

                setupAutoRoutine(routine, autoA, autoB, autoC, 2500, 0.35);
                return routine;
        }

        // ========== LEFT SIDE ROUTINES ==========

        /**
         * LEFT: Ballpit Mid Distance → LEFT Shoot
         * Starts from left, collects from middle ball zone, shoots from left
         */
        private AutoRoutine L_Bm_Ls() {
                boolean AUTO_VERIFIED = true;
                if (!AUTO_VERIFIED) {
                        System.out.println("WARNING: L_Bm_Ls routine not verified!");
                }

                final AutoRoutine routine = autoFactory.newRoutine("LEFT: Ballpit Mid → LEFT Shoot");
                final AutoTrajectory autoA = A_LeftStartlineToAlliedBallzone.asAutoTraj(routine);
                final AutoTrajectory autoB = B_LeftBallzoneToMiddleBallzone.asAutoTraj(routine);
                final AutoTrajectory autoC = C_LeftBallzoneToRightShoot.asAutoTraj(routine);

                setupAutoRoutine(routine, autoA, autoB, autoC, 2400, 0.32);
                return routine;
        }

        /**
         * LEFT: Ballpit Long Distance → LEFT Shoot
         * Starts from left, collects from far ball zone, shoots from left
         */
        private AutoRoutine L_Bl_Ls() {
                boolean AUTO_VERIFIED = true;
                if (!AUTO_VERIFIED) {
                        System.out.println("WARNING: L_Bl_Ls routine not verified!");
                }

                final AutoRoutine routine = autoFactory.newRoutine("LEFT: Ballpit Long → LEFT Shoot");
                final AutoTrajectory autoA = A_LeftStartlineToAlliedBallzone.asAutoTraj(routine);
                final AutoTrajectory autoB = B_LeftBallzoneToLeftBallzone.asAutoTraj(routine);
                final AutoTrajectory autoC = C_RightBallzoneToLeftShoot.asAutoTraj(routine);

                setupAutoRoutine(routine, autoA, autoB, autoC, 2500, 0.35);
                return routine;
        }

        // ========== CENTER-LEFT ROUTINES ==========

        // left to long and return lef

        private AutoRoutine _CL_Bl_Ls() {
                boolean AUTO_VERIFIED = false;
                if (!AUTO_VERIFIED) {
                        System.out.println("WARNING: CL_Bm_Ls routine not verified!");
                }

                final AutoRoutine routine = autoFactory.newRoutine("CENTER-LEFT: Ballpit Mid → LEFT Shoot");
                final AutoTrajectory autoA = A_LeftStartlineToAlliedBallzone.asAutoTraj(routine);
                final AutoTrajectory autoB = B_LeftBallzoneToLeftBallzone.asAutoTraj(routine);
                final AutoTrajectory autoC = C_LeftBallzoneToRightShoot.asAutoTraj(routine);

                setupAutoRoutine(routine, autoA, autoB, autoC, 2400, 0.32);
                return routine;
        }

        /**
         * CENTER-LEFT: Ballpit Mid Distance → LEFT Shoot
         * Starts from center-left, collects from middle ball zone, shoots from left
         */
        private AutoRoutine CL_Bm_Ls() {
                boolean AUTO_VERIFIED = false;
                if (!AUTO_VERIFIED) {
                        System.out.println("WARNING: CL_Bm_Ls routine not verified!");
                }

                final AutoRoutine routine = autoFactory.newRoutine("CENTER-LEFT: Ballpit Mid → LEFT Shoot");
                final AutoTrajectory autoA = A_CenterStartlineToLeftBallzone.asAutoTraj(routine);
                final AutoTrajectory autoB = B_LeftBallzoneToMiddleBallzone.asAutoTraj(routine);
                final AutoTrajectory autoC = C_LeftBallzoneToRightShoot.asAutoTraj(routine);

                setupAutoRoutine(routine, autoA, autoB, autoC, 2400, 0.32);
                return routine;
        }

        /**
         * CENTER-LEFT: Ballpit Long Distance → LEFT Shoot
         * Starts from center-left, collects from far ball zone, shoots from left
         */
        private AutoRoutine CL_Bl_Ls() {
                boolean AUTO_VERIFIED = false;
                if (!AUTO_VERIFIED) {
                        System.out.println("WARNING: CL_Bl_Ls routine not verified!");
                }

                final AutoRoutine routine = autoFactory.newRoutine("CENTER-LEFT: Ballpit Long → LEFT Shoot");
                final AutoTrajectory autoA = A_CenterStartlineToLeftBallzone.asAutoTraj(routine);
                final AutoTrajectory autoB = B_LeftBallzoneToLeftBallzone.asAutoTraj(routine);
                final AutoTrajectory autoC = C_RightBallzoneToLeftShoot.asAutoTraj(routine);

                setupAutoRoutine(routine, autoA, autoB, autoC, 2500, 0.35);
                return routine;
        }

        /**
         * CENTER-LEFT: Ballpit Long Distance → RIGHT Shoot
         * Starts from center-left, collects from far ball zone, shoots from right
         */
        private AutoRoutine CL_Bl_Rs() {
                boolean AUTO_VERIFIED = false;
                if (!AUTO_VERIFIED) {
                        System.out.println("WARNING: CL_Bl_Rs routine not verified!");
                }

                final AutoRoutine routine = autoFactory.newRoutine("CENTER-LEFT: Ballpit Long → RIGHT Shoot");
                final AutoTrajectory autoA = A_CenterStartlineToLeftRivalBallzone.asAutoTraj(routine);
                final AutoTrajectory autoB = B_LeftBallzoneToLeftBallzone.asAutoTraj(routine);
                final AutoTrajectory autoC = C_RightBallzoneToLeftShoot.asAutoTraj(routine);

                setupAutoRoutine(routine, autoA, autoB, autoC, 2500, 0.35);
                return routine;
        }

        // ========== CENTER-RIGHT ROUTINES ==========

        /**
         * CENTER-RIGHT: Ballpit Mid Distance → RIGHT Shoot
         * Starts from center-right, collects from middle ball zone, shoots from right
         */
        private AutoRoutine CR_Bm_Rs() {
                boolean AUTO_VERIFIED = false;
                if (!AUTO_VERIFIED) {
                        System.out.println("WARNING: CR_Bm_Rs routine not verified!");
                }

                final AutoRoutine routine = autoFactory.newRoutine("CENTER-RIGHT: Ballpit Mid → RIGHT Shoot");
                final AutoTrajectory autoA = A_CenterStartlineToRightBallzone.asAutoTraj(routine);
                final AutoTrajectory autoB = B_RightBallzoneToMiddleBallzone.asAutoTraj(routine);
                final AutoTrajectory autoC = C_RightBallzoneToLeftShoot.asAutoTraj(routine);

                setupAutoRoutine(routine, autoA, autoB, autoC, 2400, 0.32);
                return routine;
        }

        /**
         * CENTER-RIGHT: Ballpit Long Distance → RIGHT Shoot
         * Starts from center-right, collects from far ball zone, shoots from right
         */
        private AutoRoutine CR_Bl_Rs() {
                boolean AUTO_VERIFIED = false;
                if (!AUTO_VERIFIED) {
                        System.out.println("WARNING: CR_Bl_Rs routine not verified!");
                }

                final AutoRoutine routine = autoFactory.newRoutine("CENTER-RIGHT: Ballpit Long → RIGHT Shoot");
                final AutoTrajectory autoA = A_CenterStartlineToRivalRightBallzone.asAutoTraj(routine);
                final AutoTrajectory autoB = B_RightBallzoneToLeftBallzone.asAutoTraj(routine);
                final AutoTrajectory autoC = C_LeftBallzoneToRightShoot.asAutoTraj(routine);

                setupAutoRoutine(routine, autoA, autoB, autoC, 2500, 0.35);
                return routine;
        }

        /**
         * CENTER-RIGHT: Ballpit Mid Distance → LEFT Shoot
         * Starts from center-right, collects from middle ball zone, shoots from left
         */
        private AutoRoutine CR_Bm_Ls() {
                boolean AUTO_VERIFIED = false;
                if (!AUTO_VERIFIED) {
                        System.out.println("WARNING: CR_Bm_Ls routine not verified!");
                }

                final AutoRoutine routine = autoFactory.newRoutine("CENTER-RIGHT: Ballpit Mid → LEFT Shoot");
                final AutoTrajectory autoA = A_CenterStartlineToRightBallzone.asAutoTraj(routine);
                final AutoTrajectory autoB = B_RightBallzoneToLeftBallzone.asAutoTraj(routine);
                final AutoTrajectory autoC = C_RightBallzoneToLeftShoot.asAutoTraj(routine);

                setupAutoRoutine(routine, autoA, autoB, autoC, 2400, 0.32);
                return routine;
        }

        /**
         * CENTER-RIGHT: Ballpit Long Distance → LEFT Shoot
         * Starts from center-right, collects from far ball zone, shoots from left
         */
        private AutoRoutine CR_Bl_Ls() {
                boolean AUTO_VERIFIED = false;
                if (!AUTO_VERIFIED) {
                        System.out.println("WARNING: CR_Bl_Ls routine not verified!");
                }

                final AutoRoutine routine = autoFactory.newRoutine("CENTER-RIGHT: Ballpit Long → LEFT Shoot");
                final AutoTrajectory autoA = A_CenterStartlineToRivalRightBallzone.asAutoTraj(routine);
                final AutoTrajectory autoB = B_RightBallzoneToLeftBallzone.asAutoTraj(routine);
                final AutoTrajectory autoC = C_RightBallzoneToLeftShoot.asAutoTraj(routine);

                setupAutoRoutine(routine, autoA, autoB, autoC, 2500, 0.35);
                return routine;
        }
}
