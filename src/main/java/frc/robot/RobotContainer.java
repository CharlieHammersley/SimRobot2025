// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OIConstants;
//import frc.robot.commands.Autos;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SwerveJoystickCmd;

public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);


  public RobotContainer() {
    SmartDashboard.putBoolean("W", false);
    SmartDashboard.putBoolean("A", false);
    SmartDashboard.putBoolean("S", false);
    SmartDashboard.putBoolean("D", false);
    SmartDashboard.putBoolean("Q", false);  // turn left
    SmartDashboard.putBoolean("E", false);  // turn right

    new SwerveJoystickCmd(
        swerveSubsystem,
        
        // X (forward/back)
        () -> SmartDashboard.getBoolean("W", false) ? 1.0 :
              SmartDashboard.getBoolean("S", false) ? -1.0 : 0.0,

        // Y (strafe left/right)
        () -> SmartDashboard.getBoolean("D", false) ? 1.0 :
              SmartDashboard.getBoolean("A", false) ? -1.0 : 0.0,

        // Turning (Q = left, E = right)
        () -> SmartDashboard.getBoolean("Q", false) ? -1.0 : 
              SmartDashboard.getBoolean("E", false) ?  1.0 : 0.0,

        // Always field-oriented for now
        () -> true
    );

    /*swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem,
      () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
      () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
      () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
      () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)
      
    ));*/
    // Configure the trigger bindings
    configureBindings();
  }
  
  private void configureBindings() {
    JoystickButton zeroHeadingButton = new JoystickButton(driverJoystick, 2);
    zeroHeadingButton.onTrue(Commands.runOnce(() -> swerveSubsystem.zeroHeading(), swerveSubsystem));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
