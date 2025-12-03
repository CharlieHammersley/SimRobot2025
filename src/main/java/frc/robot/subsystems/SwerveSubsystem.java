package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.Pigeon2;

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


public class SwerveSubsystem extends SubsystemBase{

    private final Field2d field = new Field2d();
    private final ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");

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

    public SwerveSubsystem() {
        gyro.reset();
        driveTab.add("Field", field).withSize(6, 4).withPosition(0, 0);

        SmartDashboard.putData("Field", field);
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

        SmartDashboard.putNumber("Sim Gyro Heading", getHeading());
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

    @Override
    public void periodic() {
        odometry.update(
            getRotation2d(),
            new SwerveModulePosition[] {
                new SwerveModulePosition(frontLeft.getDriveDistanceMeters(), frontLeft.getTurningRotation2d()),
                new SwerveModulePosition(frontRight.getDriveDistanceMeters(), frontRight.getTurningRotation2d()),
                new SwerveModulePosition(backLeft.getDriveDistanceMeters(), backLeft.getTurningRotation2d()),
                new SwerveModulePosition(backRight.getDriveDistanceMeters(), backRight.getTurningRotation2d())
            });

        field.setRobotPose(odometry.getPoseMeters());

        SmartDashboard.putNumber("Heading Debug", getHeading());
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
