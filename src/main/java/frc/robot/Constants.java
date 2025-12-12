package frc.robot;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

// imported values, not final by any means, for Sim robot ONLY
public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * 2 * Math.PI;
        public static final double kDriveMotorGearRatio = 1 / 5.8462;
        public static final double kTurningMotorGearRatio = 1 / 18.0;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 6.0;
        public static final double kITurning = 0.0;
        public static final double kDTurning = 0.1;   // small damping
    }

    public static final class DriveConstants {
        public static final int kPigeonCanID = 2;

        public static final double kTrackWidth = Units.inchesToMeters(21);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(25.5);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        // Front Left
        new Translation2d(kWheelBase / 2.0,  kTrackWidth / 2.0),
        // Front Right
        new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
        // Back Left
        new Translation2d(-kWheelBase / 2.0,  kTrackWidth / 2.0),
        // Back Right
        new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0));



        public static final int kFrontLeftDriveMotorPort = 8;
        public static final int kBackLeftDriveMotorPort = 2;
        public static final int kFrontRightDriveMotorPort = 6;
        public static final int kBackRightDriveMotorPort = 4;

        public static final int kFrontLeftTurningMotorPort = 7;
        public static final int kBackLeftTurningMotorPort = 1;
        public static final int kFrontRightTurningMotorPort = 5;
        public static final int kBackRightTurningMotorPort = 3;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final boolean kFrontLeftTurningAbsoluteEncoderReversed  = false;
        public static final boolean kFrontRightTurningAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftTurningAbsoluteEncoderReversed   = false;
        public static final boolean kBackRightTurningAbsoluteEncoderReversed  = false;

        public static final boolean kFrontLeftDriveMotorReversed  = false;
        public static final boolean kFrontRightDriveMotorReversed = false;
        public static final boolean kBackLeftDriveMotorReversed   = false;
        public static final boolean kBackRightDriveMotorReversed  = false;

        public static final boolean kFrontLeftTurningMotorReversed  = false;
        public static final boolean kFrontRightTurningMotorReversed = false;
        public static final boolean kBackLeftTurningMotorReversed   = false;
        public static final boolean kBackRightTurningMotorReversed  = false;

        public static final int kFrontLeftTurningAbsoluteEncoderPort  = 0;
        public static final int kFrontRightTurningAbsoluteEncoderPort = 1;
        public static final int kBackLeftTurningAbsoluteEncoderPort   = 2;
        public static final int kBackRightTurningAbsoluteEncoderPort  = 3;

        public static final double kFrontLeftTurningAbsoluteEncoderOffsetRad  = -0.254;
        public static final double kFrontRightTurningAbsoluteEncoderOffsetRad = -1.816;
        public static final double kBackLeftTurningAbsoluteEncoderOffsetRad   = -1.252;
        public static final double kBackRightTurningAbsoluteEncoderOffsetRad  = -4.811;


        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond =
                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond =
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.05;
    }
}