package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandBase;


public class SwerveJoystickCmd extends CommandBase{
    
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<boolean> feildOrientatedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem, // gets the joystick values in terms of Suppliers and a boolean for if the user wants the robot to drive feild relative or not
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<double> turningSpdFunction,
            Supplier<boolean> feildOrientatedFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.feildOrientatedFunction = feildOrientatedFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.tunringLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // get realtime joystick inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();
        
        // apply deadband -- if the joystick doesn't center back to eactly zero we will ignore any small inputs
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? tunringSpeed : 0.0;

        // rate limiter - limits joysick input from being too violent -- limit accelleration to ensure smooth driving
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if(feildOrientatedFunction.get()) {
            chassisSpeeds = ChassisSpeeds.fromFeildRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // convert chassis to individual speeds
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // output each state to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
