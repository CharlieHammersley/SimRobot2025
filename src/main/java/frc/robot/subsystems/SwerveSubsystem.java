package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;


public class SwerveSubsystem extends SubsystemBase{

    private final Field2d field = new Field2d();

    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort, // drive motor port
            DriveConstants.kFrontLeftTurningMotorPort, // turning motor port
            DriveConstants.kFrontLeftDriveEncoderReversed, // drive encoder
            DriveConstants.kFrontLeftTurningEncoderReversed, // turning encoder
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort, // drive encoder port
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad, 
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private final Pigeon2 gyro = new Pigeon2(DriveConstants.kPigeonCanID); // navx gyro

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        getRotation2d(),
        new SwerveModulePosition[] {
            new SwerveModulePosition(frontLeft.getDriveDistanceMeters(), frontLeft.getTurningRotation2d()),
            new SwerveModulePosition(frontRight.getDriveDistanceMeters(), frontRight.getTurningRotation2d()),
            new SwerveModulePosition(backLeft.getDriveDistanceMeters(), backLeft.getTurningRotation2d()),
            new SwerveModulePosition(backRight.getDriveDistanceMeters(), backRight.getTurningRotation2d())
        });

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            new SwerveModulePosition(frontLeft.getDriveDistanceMeters(), frontLeft.getTurningRotation2d()),
            new SwerveModulePosition(frontRight.getDriveDistanceMeters(), frontRight.getTurningRotation2d()),
            new SwerveModulePosition(backLeft.getDriveDistanceMeters(), backLeft.getTurningRotation2d()),
            new SwerveModulePosition(backRight.getDriveDistanceMeters(), backRight.getTurningRotation2d())
        };
    }

    public SwerveSubsystem() {
        gyro.reset();

        SmartDashboard.putData("Field", field);
        SmartDashboard.putData("Swerve States", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");
                builder.addDoubleArrayProperty("States",
                    () -> new double[]{
                        frontLeft.getState().speedMetersPerSecond, frontLeft.getState().angle.getRadians(),
                        frontRight.getState().speedMetersPerSecond, frontRight.getState().angle.getRadians(),
                        backLeft.getState().speedMetersPerSecond, backLeft.getState().angle.getRadians(),
                        backRight.getState().speedMetersPerSecond, backRight.getState().angle.getRadians()
                    },
                    null
                );
            }
        });
    }


    @Override
    public void simulationPeriodic() {
        frontLeft.simulationUpdate();
        frontRight.simulationUpdate();
        backLeft.simulationUpdate();
        backRight.simulationUpdate();

        var chassisSpeeds = DriveConstants.kDriveKinematics.toChassisSpeeds(
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        );

        double dt = 0.02;
        double deltaYawDeg = Math.toDegrees(chassisSpeeds.omegaRadiansPerSecond * dt);
        gyro.getSimState().addYaw(deltaYawDeg);
    }

    public void zeroHeading() { // sets the zero heading to wherever the robot was facing when it was booted up
        gyro.reset();
    }

    public double getHeading() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()).getDegrees();
        //return Math.IEEEremainder(gyro.getYaw().getValueAsDouble(),360.0);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
            getRotation2d(),
            getModulePositions(),
            pose
        );
    }
    
    public void resetToCenter() {
        Pose2d centerPose = new Pose2d(8.27, 4.105, Rotation2d.fromDegrees(0));
        odometry.resetPosition(getRotation2d(), getModulePositions(), centerPose);
        field.setRobotPose(centerPose);
    }

    @Override
    public void periodic() {
        odometry.update(getRotation2d(), getModulePositions());
        field.setRobotPose(odometry.getPoseMeters());

        //field.setRobotPose(getPose());

        SmartDashboard.putNumber("Robot Velocity (m/s)",DriveConstants.kDriveKinematics.toChassisSpeeds(
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()).vxMetersPerSecond);

        /*odometry.update(
            getRotation2d(),
            new SwerveModulePosition[] {
                new SwerveModulePosition(frontLeft.getDriveDistanceMeters(), frontLeft.getTurningRotation2d()),
                new SwerveModulePosition(frontRight.getDriveDistanceMeters(), frontRight.getTurningRotation2d()),
                new SwerveModulePosition(backLeft.getDriveDistanceMeters(), backLeft.getTurningRotation2d()),
                new SwerveModulePosition(backRight.getDriveDistanceMeters(), backRight.getTurningRotation2d())
            });

        field.setRobotPose(odometry.getPoseMeters());

        SmartDashboard.putNumber("Heading Debug", getHeading());
        field.setRobotPose(odometry.getPoseMeters());*/
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}
