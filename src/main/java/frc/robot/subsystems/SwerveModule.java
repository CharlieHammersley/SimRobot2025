package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;


public class SwerveModule {
    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    private TalonFXSimState driveSim;
    private TalonFXSimState turnSim;
    
    

    //private final CANcoder driveEncoder;
    //private final CANcoder turningEncoder;

    private final PIDController turningPidController;

    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    private final PositionVoltage drivePosControl = new PositionVoltage(0);
    private final PositionVoltage turnPosControl = new PositionVoltage(0);
    //private final DutyCycleOut driveOut = new DutyCycleOut(0.0);
    //private final DutyCycleOut turnOut = new DutyCycleOut(0.0);

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderId);

        driveMotor = new TalonFX(driveMotorId);
        turningMotor = new TalonFX(turningMotorId);

        driveSim = driveMotor.getSimState();
        turnSim = turningMotor.getSimState();

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.Inverted = driveMotorReversed ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        driveMotor.getConfigurator().apply(driveConfig);

        TalonFXConfiguration turningConfig = new TalonFXConfiguration();
        turningConfig.MotorOutput.Inverted = turningMotorReversed ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        turningMotor.getConfigurator().apply(turningConfig);


        //driveMotor.setInverted(driveMotorReversed);
        //turningMotor.setInverted(turningMotorReversed);

        //driveEncoder = driveMotor.getEncoder();
        //turningEncoder = turningMotor.getEncoder();

        turningConfig.Feedback.SensorToMechanismRatio = ModuleConstants.kTurningMotorGearRatio / (2.0 * Math.PI);
        turningMotor.getConfigurator().apply(turningConfig);

        //driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        //driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        //turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        //turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        driveMotor.getConfigurator().refresh(driveConfig);
        driveConfig.Feedback.SensorToMechanismRatio = ModuleConstants.kDriveMotorGearRatio / ModuleConstants.kWheelCircumferenceMeters;
        driveMotor.getConfigurator().apply(driveConfig);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }
    

    public double getDriveDistanceMeters() {
        return driveMotor.getPosition().getValueAsDouble();
    }
    
    public Rotation2d getTurningRotation2d() {
        return Rotation2d.fromRadians(turningMotor.getPosition().getValueAsDouble());
    }

    public double getDrivePosition() {
        return driveMotor.getPosition().getValueAsDouble();
    }

    public double getTurningPosition() {
        return turningMotor.getPosition().getValueAsDouble();
    }

    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValueAsDouble();
    }

    public double getTurningVelocity() {
        return turningMotor.getVelocity().getValueAsDouble();
    }

    public double getAbsoluteEncoderRad() {
    
        if (Robot.isSimulation()) {
            return turningMotor.getPosition().getValueAsDouble(); // fake it
        }
        
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() { // sets encoder to absolute position
        driveMotor.setControl(drivePosControl.withPosition(0.0));
        turningMotor.setControl(turnPosControl.withPosition(getAbsoluteEncoderRad()));
    }
    

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) { // stops module from reseting 
            stop();
            return;
        }

        // optimize so module never has to turn more than 90 degrees
        state.optimize(getState().angle);

        // sets both motors to the calculated values
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    }


    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    public void simulationUpdate() {
        double dt = 0.02;
    
        driveSim.setSupplyVoltage(12.0);
        turnSim.setSupplyVoltage(12.0);

        // convert command percent to wheel speed
        double appliedDutyCycle = driveMotor.getDutyCycle().getValueAsDouble();
        double wheelSpeed = appliedDutyCycle * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;

        // wheel speed --> motor rot/sec
        double motorRPS = wheelSpeed / ModuleConstants.kWheelCircumferenceMeters * ModuleConstants.kDriveMotorGearRatio;

        // set simulation velocity
        driveSim.setRotorVelocity(motorRPS);

        driveSim.addRotorPosition(motorRPS * dt);

        double turnDuty = turningMotor.getDutyCycle().getValueAsDouble();
        double turnSpeedRad = turnDuty * 10.0; // fake speed, change

        //double newPos = turnSim.getRotorPosition() + turnSpeedRad * dt;

        double turnRPS = turnSpeedRad / (2.0 * Math.PI);

        turnSim.setRotorVelocity(turnRPS);
        turnSim.addRotorPosition(turnRPS * dt);

    }
    
}
