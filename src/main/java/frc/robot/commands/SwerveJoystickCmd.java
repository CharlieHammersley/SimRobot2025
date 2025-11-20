package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandBase;


public class SwerveJoystickCmd extends CommandBase{
    
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<boolean> feildOrientatedFunction;

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem, // gets the joystick values in terms of Suppliers and a boolean for if the user wants the robot to drive feild relative or not
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<double> turningSpdFunction,
            Supplier<boolean> feildOrientatedFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.feildOrientatedFunction = feildOrientatedFunction;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // get realtime inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();
        // (12:35)
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
