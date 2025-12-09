// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OIConstants;
//import frc.robot.commands.Autos;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SwerveJoystickCmd;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Joystick driverJoystick = new Joystick(0);


  public RobotContainer() {
    configureBindings();
    swerveSubsystem.setDefaultCommand(
      new RunCommand(() -> {
          double xSpeed = -driverJoystick.getRawAxis(1) * 2.5; // Forward/back
          double ySpeed = -driverJoystick.getRawAxis(0) * 2.5; // Strafe
          double rot    = -driverJoystick.getRawAxis(2) * 2.0; // Rotation (wheel or trigger)

          var states = DriveConstants.kDriveKinematics.toSwerveModuleStates(
              new ChassisSpeeds(xSpeed, ySpeed, rot)
          );
          swerveSubsystem.setModuleStates(states);
      }, swerveSubsystem));
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
    return null;
  }
}
