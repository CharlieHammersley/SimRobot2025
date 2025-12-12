package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.AnalogInput;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Robot;

public class SwerveModule {
    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    private final TalonFXSimState driveSim;
    private final TalonFXSimState turnSim;

    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    private final PIDController turningPid;

    private final PositionVoltage drivePosControl = new PositionVoltage(0);
    private final PositionVoltage turnPosControl = new PositionVoltage(0);

    public SwerveModule(int driveMotorId,
                        int turningMotorId,
                        boolean driveMotorReversed,
                        boolean turningMotorReversed,
                        int absoluteEncoderPort,
                        double absoluteEncoderOffset,
                        boolean absoluteEncoderReversed) {
                        
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        this.absoluteEncoder = (absoluteEncoderPort >= 0) ? new AnalogInput(absoluteEncoderPort) : null;

        driveMotor = new TalonFX(driveMotorId);
        turningMotor = new TalonFX(turningMotorId);

        driveMotor.getConfigurator().apply(new com.ctre.phoenix6.configs.TalonFXConfiguration() {{
            MotorOutput.Inverted = driveMotorReversed ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        }});
        turningMotor.getConfigurator().apply(new com.ctre.phoenix6.configs.TalonFXConfiguration() {{
            MotorOutput.Inverted = turningMotorReversed ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
            Feedback.SensorToMechanismRatio = ModuleConstants.kTurningMotorGearRatio / (2.0 * Math.PI);
        }});

        driveSim = driveMotor.getSimState();
        turnSim = turningMotor.getSimState();

        turningPid = new PIDController(ModuleConstants.kPTurning, ModuleConstants.kITurning, ModuleConstants.kDTurning);
        turningPid.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    private static double wrapAngleRadians(double angle) {
        while (angle >= Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    private double getTurningAngleRadFromIntegrated() {
        double motorRotations = turningMotor.getPosition().getValueAsDouble();
        return motorRotations * ModuleConstants.kTurningEncoderRot2Rad;
    }

    private double getDriveDistanceMetersFromIntegrated() {
        double motorRotations = driveMotor.getPosition().getValueAsDouble();
        return motorRotations * ModuleConstants.kDriveEncoderRot2Meter;
    }

    private double getDriveVelocityMetersPerSecondFromIntegrated() {
        double motorRps = driveMotor.getVelocity().getValueAsDouble();
        return motorRps * ModuleConstants.kDriveEncoderRot2Meter;
    }

    public double getAbsoluteEncoderRad() {
        if (Robot.isSimulation() || absoluteEncoder == null) {
            return wrapAngleRadians(getTurningAngleRadFromIntegrated());
        } else {
            double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V() * 2.0 * Math.PI;
            angle -= absoluteEncoderOffsetRad;
            return wrapAngleRadians(absoluteEncoderReversed ? -angle : angle);
        }
    }

    public double getDriveDistanceMeters() {
        return getDriveDistanceMetersFromIntegrated();
    }

    public double getDriveVelocityMetersPerSecond() {
        return getDriveVelocityMetersPerSecondFromIntegrated();
    }

    public Rotation2d getTurningRotation2d() {
        return new Rotation2d(getAbsoluteEncoderRad());
    }

    public double getTurningPositionMotorRotations() {
        return turningMotor.getPosition().getValueAsDouble();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocityMetersPerSecond(), getTurningRotation2d());
    }

    public void resetEncoders() {
        driveMotor.setControl(drivePosControl.withPosition(0.0));

        double initTurnRad = getAbsoluteEncoderRad();
        double initMotorRot = initTurnRad / ModuleConstants.kTurningEncoderRot2Rad;
        turningMotor.setControl(turnPosControl.withPosition(initMotorRot));
    }

    public void stop() {
        driveMotor.set(0.0);
    
        double currentMotorRot = turningMotor.getPosition().getValueAsDouble();
        turningMotor.setControl(turnPosControl.withPosition(currentMotorRot));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState d = new SwerveModuleState(
            Math.min(desiredState.speedMetersPerSecond, DriveConstants.kPhysicalMaxSpeedMetersPerSecond),
            desiredState.angle);
    
        d = SwerveModuleState.optimize(d, getState().angle);
    
        double percentOutput = d.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        driveMotor.set(percentOutput);
    
        
        double currentAngle = getAbsoluteEncoderRad();
        double targetAngle = wrapAngleRadians(d.angle.getRadians());
        
        final double ANGLE_DEADBAND_RAD = 0.005;
        final double SPEED_DEADBAND = 0.005; 
    
        if (Math.abs(d.speedMetersPerSecond) < SPEED_DEADBAND && Math.abs(wrapAngleRadians(targetAngle - currentAngle)) < ANGLE_DEADBAND_RAD) {
            
            double holdMotorRot = turningMotor.getPosition().getValueAsDouble();
            turningMotor.setControl(turnPosControl.withPosition(holdMotorRot));
            
            return;
        }
    
        double targetMotorRot = targetAngle / ModuleConstants.kTurningEncoderRot2Rad;
    
        turningMotor.setControl(turnPosControl.withPosition(targetMotorRot));
    }

    public void simulationUpdate() {
        final double dt = 0.02;

        driveSim.setSupplyVoltage(12.0);
        turnSim.setSupplyVoltage(12.0);

        double driveDuty = driveMotor.getDutyCycle().getValueAsDouble();
        double wheelSpeed = driveDuty * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;

        double wheelRps = wheelSpeed / ModuleConstants.kWheelCircumferenceMeters;
        double motorRps = wheelRps / ModuleConstants.kDriveMotorGearRatio;

        driveSim.setRotorVelocity(motorRps);
        driveSim.addRotorPosition(motorRps * dt);

        double turnDuty = turningMotor.getDutyCycle().getValueAsDouble();
        double maxTurnRadPerSec = 2.0 * Math.PI;
        double turnSpeedRadPerSec = turnDuty * maxTurnRadPerSec;
        double turnMotorRps = turnSpeedRadPerSec / (2.0 * Math.PI) / ModuleConstants.kTurningMotorGearRatio;
        turnSim.setRotorVelocity(turnMotorRps);
        turnSim.addRotorPosition(turnMotorRps * dt);
    }
}
