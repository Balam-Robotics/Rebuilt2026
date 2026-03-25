package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

public class Swerve extends TunerSwerveDrivetrain implements Subsystem {

    private static final Rotation2d kBLUEALLIANCEPERS_ROTATION2D = Rotation2d.kZero;
    private static final Rotation2d kREDALLIANCEPERS_ROTATION2D = Rotation2d.k180deg;
    private boolean m_haveFlipped = false;
    public boolean m_fieldRelative = true;

    private final SwerveRequest.ApplyFieldSpeeds pathFieldSpeedsRequest = new SwerveRequest.ApplyFieldSpeeds();
    private final PIDController pathXController = new PIDController(10, 0, 0);
    private final PIDController pathYController = new PIDController(10, 0, 0);
    private final PIDController pathThetaController = new PIDController(7.5, 0, 0);

    private Field2d m_field = new Field2d();

    public Swerve() {
        super(
                TunerConstants.DrivetrainConstants,
                0,
                VecBuilder.fill(0.1, 0.1, 0.1),
                VecBuilder.fill(0.1, 0.1, 0.1),
                TunerConstants.FrontLeft,
                TunerConstants.FrontRight,
                TunerConstants.BackLeft,
                TunerConstants.BackRight);
        SmartDashboard.putData("Traj Field", m_field);
    }

    public AutoFactory createAutoFactory() {
        return createAutoFactory((sample, isStart) -> {
        });
    }

    public AutoFactory createAutoFactory(TrajectoryLogger<SwerveSample> trajectoryLogger) {
        return new AutoFactory(
                () -> getState().Pose,
                this::resetPose,
                this::followPath,
                true,
                this,
                trajectoryLogger);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command flipRobotMode() {
        return runOnce(() -> {
            m_fieldRelative = !m_fieldRelative;
            if (!m_fieldRelative) {
                setOperatorPerspectiveForward(getState().Pose.getRotation()); // chacne tengo q quitar eso
            }
        });
    }

    public void followPath(SwerveSample sample) {
        pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

        var pose = getState().Pose;

        var targetSpeeds = sample.getChassisSpeeds();
        targetSpeeds.vxMetersPerSecond += pathXController.calculate(
                pose.getX(), sample.x);
        targetSpeeds.vyMetersPerSecond += pathYController.calculate(
                pose.getY(), sample.y);
        targetSpeeds.omegaRadiansPerSecond += pathThetaController.calculate(
                pose.getRotation().getRadians(), sample.heading);

        // display where the robot SHOULD Be with each sample in the m_field
        Pose2d targetPose = new Pose2d(
                sample.x,
                sample.y,
                new Rotation2d(sample.heading));

        m_field.getObject("target").setPose(targetPose);

        setControl(
                pathFieldSpeedsRequest.withSpeeds(targetSpeeds)
                        .withWheelForceFeedforwardsX(sample.moduleForcesX())
                        .withWheelForceFeedforwardsY(sample.moduleForcesY()));
    }

    @Override
    public void periodic() {
        if (!m_haveFlipped || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? kREDALLIANCEPERS_ROTATION2D : kBLUEALLIANCEPERS_ROTATION2D);
                if (!m_haveFlipped) {
                    seedFieldCentric();
                }
                m_haveFlipped = true;
            });
        }
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
    }

    public Field2d getField() {
        return m_field;
    }
}
