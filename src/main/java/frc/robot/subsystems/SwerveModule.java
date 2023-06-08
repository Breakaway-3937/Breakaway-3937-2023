package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

public class SwerveModule {
    public int moduleNumber;
    private double angleOffset;
    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;
    private double lastAngle;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Drivetrain.DRIVE_KS, Constants.Drivetrain.DRIVE_KV, Constants.Drivetrain.DRIVE_KA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID, "CANivore");
        configAngleEncoder();
        
        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID, "CANivore");
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID, "CANivore");
        configDriveMotor();

        lastAngle = getState().angle.getDegrees();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); //Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not

        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Drivetrain.MAX_SPEED;
            mDriveMotor.setControl(new DutyCycleOut(percentOutput, true, false));
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Drivetrain.WHEEL_CIRCUMFERENCE, Constants.Drivetrain.DRIVE_GEAR_RATIO);
            mDriveMotor.setControl(new VelocityDutyCycle(velocity, true, 0, 0, false));
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Drivetrain.MAX_SPEED * 0.01)) ? lastAngle : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        mAngleMotor.setControl(new PositionVoltage(Conversions.degreesToFalcon(angle, Constants.Drivetrain.ANGLE_GEAR_RATIO), true, 0, 0, false)); 
        lastAngle = angle;
    }

    private void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset, Constants.Drivetrain.ANGLE_GEAR_RATIO);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        //angleEncoder.configFactoryDefault();
        angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
        //angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        //mAngleMotor.configFactoryDefault();
        mAngleMotor.getConfigurator().apply(new TalonFXConfiguration()); //New default method
        //mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        mAngleMotor.setInverted(Constants.Drivetrain.ANGLE_MOTOR_INVERT);
        //mAngleMotor.setNeutralMode(Constants.Drivetrain.ANGLE_NEUTRAL_MODE); //IN CTRE CONFIG NOW
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        //mDriveMotor.configFactoryDefault();
        mDriveMotor.getConfigurator().apply(new TalonFXConfiguration()); //New default method
        //mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setInverted(Constants.Drivetrain.DRIVE_MOTOR_INVERT);
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        //mDriveMotor.setNeutralMode(Constants.Drivetrain.DRIVE_NEUTRAL_MODE); //IN CTRE CONFIG NOW
        mDriveMotor.setSelectedSensorPosition(0);
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValue());
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), Constants.Drivetrain.ANGLE_GEAR_RATIO));
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.falconToMeters(mDriveMotor.getSelectedSensorPosition(), Constants.Drivetrain.WHEEL_CIRCUMFERENCE, Constants.Drivetrain.DRIVE_GEAR_RATIO), 
            getAngle()
        );
    }

    public SwerveModuleState getState(){
        double velocity = Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), Constants.Drivetrain.WHEEL_CIRCUMFERENCE, Constants.Drivetrain.DRIVE_GEAR_RATIO);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), Constants.Drivetrain.ANGLE_GEAR_RATIO));
        return new SwerveModuleState(velocity, angle);
    }
    
}