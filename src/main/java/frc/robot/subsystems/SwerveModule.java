package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Robot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

public class SwerveModule {
    public int moduleNumber;
    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;
    private StatusSignal<Double> drivePosition;
    private StatusSignal<Double> driveVelocity;
    private StatusSignal<Double> anglePosition;
    private StatusSignal<Double> angleVelocity;
    private double rotationsPerMeter = Constants.Drivetrain.DRIVE_GEAR_RATIO / (2 * Math.PI * Constants.Drivetrain.WHEEL_DIAMETER);
    private VelocityTorqueCurrentFOC  torqueCurrentFOC = new VelocityTorqueCurrentFOC (0,0,1,false);


    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID, "CANivore");
        configAngleEncoder(moduleConstants.angleOffset);
        
        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID, "CANivore");
        configAngleMotor(moduleConstants.cancoderID);

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID, "CANivore");
        configDriveMotor();

        drivePosition = mDriveMotor.getPosition();
        driveVelocity = mDriveMotor.getVelocity();
        anglePosition = mAngleMotor.getPosition();
        angleVelocity = mAngleMotor.getVelocity();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean focEnabled){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Drivetrain.MAX_SPEED;
            mDriveMotor.setControl(new DutyCycleOut(percentOutput, true, false));
        }
        else {
            double velocity = desiredState.speedMetersPerSecond * rotationsPerMeter;
            if(!focEnabled){
            mDriveMotor.setControl(new VelocityDutyCycle(velocity, false, 0, 0, false));
            }
            else if(focEnabled){
            ///mDriveMotor.setControl(torqueCurrentFOC.withVelocity(velocity).withSlot(1));
            mDriveMotor.setControl(new VelocityDutyCycle(velocity, true, 0, 0, false));
            }
        }

        mAngleMotor.setControl(new PositionVoltage(desiredState.angle.getRotations(), true, 0, 0, false)); 
    }

    private void configAngleEncoder(double offset){        
        angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
        var config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = offset;
        angleEncoder.getConfigurator().apply(config);
    }

    private void configAngleMotor(int id){
        mAngleMotor.getConfigurator().apply(new TalonFXConfiguration());
        var config = Robot.ctreConfigs.swerveAngleFXConfig;
        config.Feedback.FeedbackRemoteSensorID = id;
        mAngleMotor.getConfigurator().apply(config);
    }

    private void configDriveMotor(){        
        mDriveMotor.getConfigurator().apply(new TalonFXConfiguration());
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        mDriveMotor.setRotorPosition(0);
    }

    public double getCanCoder(){
        return angleEncoder.getAbsolutePosition().getValue();
    }

    private Rotation2d getAngle(){
        return getState().angle;
    }

    public SwerveModulePosition getPosition(){
        drivePosition.refresh();
        driveVelocity.refresh();
        return new SwerveModulePosition(
            BaseStatusSignal.getLatencyCompensatedValue(drivePosition, driveVelocity) / rotationsPerMeter, 
            getAngle()
        );
    }

    public SwerveModuleState getState(){
        anglePosition.refresh();
        angleVelocity.refresh();
        Rotation2d angle = Rotation2d.fromRotations(BaseStatusSignal.getLatencyCompensatedValue(anglePosition, angleVelocity));
        return new SwerveModuleState(0, angle);
    }
    
}