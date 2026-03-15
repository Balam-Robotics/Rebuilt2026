package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;
import com.ctre.phoenix6.CANBus;

import frc.robot.generated.TunerConstants;

public final class Constants {

    public static class KrakenX60 {
        public static final AngularVelocity kFreeSpeed = RPM.of(6000);
    }

    public static class SwerveConstants {
        public static final LinearVelocity kMaxSpeed = TunerConstants.kSpeedAt12Volts;
        public static final AngularVelocity kMaxRotationalRate = RotationsPerSecond.of(1);
        public static final AngularVelocity kPIDRotationDeadband = kMaxRotationalRate.times(0.005);

        public static final int kPigeonID = 0;
        public static final CANBus kCANBus = new CANBus("BALAM Swerve", "./logs/example.hoot");

        // Front Left
        public static final int kFrontLeftDriveMotorId = 1;
        public static final int kFrontLeftSteerMotorId = 2;
        public static final int kFrontLeftEncoderId = 3;
        public static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.412109375);
        public static final boolean kFrontLeftSteerMotorInverted = true;
        public static final boolean kFrontLeftEncoderInverted = false;

        // Front Right
        public static final int kFrontRightDriveMotorId = 11;
        public static final int kFrontRightSteerMotorId = 12;
        public static final int kFrontRightEncoderId = 13;
        public static final Angle kFrontRightEncoderOffset = Rotations.of(0.3447265625);
        public static final boolean kFrontRightSteerMotorInverted = true;
        public static final boolean kFrontRightEncoderInverted = false;

        // Back Left
        public static final int kBackLeftDriveMotorId = 21;
        public static final int kBackLeftSteerMotorId = 22;
        public static final int kBackLeftEncoderId = 23;
        public static final Angle kBackLeftEncoderOffset = Rotations.of(-0.25830078125);
        public static final boolean kBackLeftSteerMotorInverted = true;
        public static final boolean kBackLeftEncoderInverted = false;

        // Back Right
        public static final int kBackRightDriveMotorId = 31;
        public static final int kBackRightSteerMotorId = 32;
        public static final int kBackRightEncoderId = 33;
        public static final Angle kBackRightEncoderOffset = Rotations.of(-0.12841796875);
        public static final boolean kBackRightSteerMotorInverted = true;
        public static final boolean kBackRightEncoderInverted = false;

    }

    public static class Ports {
        public static final CANBus kRoboRioCANBus = new CANBus("rio");

        // Talon FX IDs
        public static final int kIntakePivot = 41;
        public static final int kIntakeRollers = 42;
            
        public static final int kFloor = 51;
        public static final int kFeeder = 52;

        public static final int kShooterLeft = 61;
        public static final int kShooterMiddle = 62;
        public static final int kShooterRight = 63;

        public static final int kHanger = 18;

        // PWM 
        public static final int kHoodLeftServo = 3;
        public static final int kHoodRightServo = 4;
    }
}