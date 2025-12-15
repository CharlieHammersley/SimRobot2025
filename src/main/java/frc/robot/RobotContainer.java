// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import frc.robot.commands.SwerveJoystickCmd;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.List;
import edu.wpi.first.units.Units;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants.OIConstants;
//import frc.robot.commands.Autos;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Joystick driverJoystick = new Joystick(0);

  private final SlewRateLimiter xLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);


  public RobotContainer() {
  configureBindings();

  swerveSubsystem.setDefaultCommand(
    new RunCommand(() -> {
      SmartDashboard.putNumber("X Raw", driverJoystick.getRawAxis(0));
      SmartDashboard.putNumber("Y Raw", driverJoystick.getRawAxis(1));
      SmartDashboard.putNumber("Rot Raw", driverJoystick.getRawAxis(2));

      double xInput = applyDeadband(-driverJoystick.getRawAxis(1), 0.05);
      double yInput = applyDeadband(-driverJoystick.getRawAxis(0), 0.05);
      double rotInput = applyDeadband(-driverJoystick.getRawAxis(2), 0.05);

      double xSpeed = xLimiter.calculate(xInput) * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
      double ySpeed = yLimiter.calculate(yInput) * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
      double rot = rotLimiter.calculate(rotInput) * DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond;

      ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeed, ySpeed, rot, swerveSubsystem.getRotation2d()
      );

      var states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

      swerveSubsystem.setModuleStates(states);
    }, swerveSubsystem));
}
  private double applyDeadband(double value, double deadband) {
    return (Math.abs(value) < deadband) ? 0.0 : value;
  }
  
  private void configureBindings() {
    JoystickButton zeroHeadingButton = new JoystickButton(driverJoystick, 2);
    zeroHeadingButton.onTrue(Commands.runOnce(() -> swerveSubsystem.zeroHeading(), swerveSubsystem));
    
    new JoystickButton(driverJoystick, 1)
      .onTrue(Commands.runOnce(() ->
        swerveSubsystem.resetOdometry(new Pose2d(2.0, 4.0, new Rotation2d()))
    ));
  }

  public Command getAutonomousCommand() {
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                    AutoConstants.kMaxSpeedMetersPerSecond, 
                    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                    .setKinematics(DriveConstants.kDriveKinematics);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        List.of(new Translation2d(1, 1), new Translation2d(1, -1)),
        new Pose2d(2, -1, Rotation2d.fromDegrees(0)),
        trajectoryConfig);

    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory, 
        swerveSubsystem::getPose, 
        DriveConstants.kDriveKinematics, 
        xController, 
        yController, 
        thetaController,
        swerveSubsystem::setModuleStates,
        swerveSubsystem);

    return new SequentialCommandGroup(new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())), 
                                                        swerveControllerCommand,
                                                        new InstantCommand(() -> swerveSubsystem.stopModules()));

  }
}
