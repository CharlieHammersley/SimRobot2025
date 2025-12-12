package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;


public class SwerveSubsystem extends SubsystemBase{

    private boolean fieldRelative = false;

    private final BooleanPublisher fieldRelPublisher =
        NetworkTableInstance.getDefault().getBooleanTopic("/Swerve/FieldRelative").publish();

    private final BooleanSubscriber fieldRelSubscriber =
        NetworkTableInstance.getDefault().getBooleanTopic("/Swerve/FieldRelative").subscribe(fieldRelative);

    private final Field2d field = new Field2d();

    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftDriveMotorReversed,
        DriveConstants.kFrontLeftTurningMotorReversed,
        DriveConstants.kFrontLeftTurningAbsoluteEncoderPort,
        DriveConstants.kFrontLeftTurningAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontLeftTurningAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kFrontRightDriveMotorReversed,
        DriveConstants.kFrontRightTurningMotorReversed,
        DriveConstants.kFrontRightTurningAbsoluteEncoderPort,
        DriveConstants.kFrontRightTurningAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontRightTurningAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kBackLeftDriveMotorReversed,
        DriveConstants.kBackLeftTurningMotorReversed,
        DriveConstants.kBackLeftTurningAbsoluteEncoderPort,
        DriveConstants.kBackLeftTurningAbsoluteEncoderOffsetRad,
        DriveConstants.kBackLeftTurningAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightTurningMotorPort,
        DriveConstants.kBackRightDriveMotorReversed,
        DriveConstants.kBackRightTurningMotorReversed,
        DriveConstants.kBackRightTurningAbsoluteEncoderPort,
        DriveConstants.kBackRightTurningAbsoluteEncoderOffsetRad,
        DriveConstants.kBackRightTurningAbsoluteEncoderReversed);

    private final Pigeon2 gyro = new Pigeon2(DriveConstants.kPigeonCanID); // navx gyro

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        getRotation2d(), getModulePositions());

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
        SmartDashboard.putData("Swerve Drive", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
              builder.setSmartDashboardType("SwerveDrive");
          
              builder.addDoubleProperty("Front Left Angle",() -> frontLeft.getState().angle.getRadians(),null);
              builder.addDoubleProperty("Front Left Velocity",() -> frontLeft.getState().speedMetersPerSecond,null);
          
              builder.addDoubleProperty("Front Right Angle",() -> frontRight.getState().angle.getRadians(),null);
              builder.addDoubleProperty("Front Right Velocity",() -> frontRight.getState().speedMetersPerSecond,null);
          
              builder.addDoubleProperty("Back Left Angle",() -> backLeft.getState().angle.getRadians(),null);
              builder.addDoubleProperty("Back Left Velocity",() -> backLeft.getState().speedMetersPerSecond,null);
          
              builder.addDoubleProperty("Back Right Angle",() -> backRight.getState().angle.getRadians(),null);
              builder.addDoubleProperty("Back Right Velocity",() -> backRight.getState().speedMetersPerSecond,null);
          
              builder.addDoubleProperty("Robot Angle",() -> getRotation2d().getRadians(),null);
            }});
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

        double deltaYawDeg = Math.toDegrees(chassisSpeeds.omegaRadiansPerSecond * 0.02);
        gyro.getSimState().addYaw(deltaYawDeg);
    }

    public void zeroHeading() { // sets the zero heading to wherever the robot was facing when it was booted up
        gyro.reset();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(-gyro.getYaw().getValueAsDouble());
    }
    
    public double getHeading() {
        return getRotation2d().getDegrees();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
    }
    
    public void resetToCenter() {
        Pose2d centerPose = new Pose2d(8.27, 4.105, Rotation2d.fromDegrees(0));
        odometry.resetPosition(getRotation2d(), getModulePositions(), centerPose);
        field.setRobotPose(centerPose);
    }

    @Override
    public void periodic() {
        fieldRelative = fieldRelSubscriber.get();
        fieldRelPublisher.set(fieldRelative);

        odometry.update(getRotation2d(), getModulePositions());
        field.setRobotPose(odometry.getPoseMeters());
        
        odometry.update(getRotation2d(), getModulePositions());
        field.setRobotPose(odometry.getPoseMeters());

        SmartDashboard.putNumber("Robot Velocity (m/s)",DriveConstants.kDriveKinematics.toChassisSpeeds(
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()).vxMetersPerSecond);
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
