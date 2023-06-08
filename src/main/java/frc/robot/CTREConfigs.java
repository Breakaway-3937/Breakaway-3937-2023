package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANcoderConfiguration swerveCanCoderConfig;
    private CurrentLimitsConfigs angleSupplyLimit;
    private CurrentLimitsConfigs driveSupplyLimit;

    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANcoderConfiguration();
        angleSupplyLimit = swerveAngleFXConfig.CurrentLimits;
        driveSupplyLimit = swerveDriveFXConfig.CurrentLimits;

        /* Swerve Angle Motor Configurations */
        /*SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.Drivetrain.ANGLE_ENABLE_CURRENT_LIMIT, 
            Constants.Drivetrain.ANGLE_CONTINUOUS_CURRENT_LIMIT, 
            Constants.Drivetrain.ANGLE_PEAK_CURRENT_LIMIT, 
            Constants.Drivetrain.ANGLE_PEAK_CURRENT_DURATION);*/

            angleSupplyLimit.SupplyCurrentLimitEnable = Constants.Drivetrain.ANGLE_ENABLE_CURRENT_LIMIT;
            angleSupplyLimit.SupplyCurrentLimit = Constants.Drivetrain.ANGLE_CONTINUOUS_CURRENT_LIMIT;
            angleSupplyLimit.SupplyCurrentThreshold = Constants.Drivetrain.ANGLE_PEAK_CURRENT_LIMIT;
            angleSupplyLimit.SupplyTimeThreshold = Constants.Drivetrain.ANGLE_PEAK_CURRENT_DURATION;

        swerveAngleFXConfig.Slot0.kP = Constants.Drivetrain.ANGLE_KP;
        swerveAngleFXConfig.Slot0.kI = Constants.Drivetrain.ANGLE_KI;
        swerveAngleFXConfig.Slot0.kD = Constants.Drivetrain.ANGLE_KD;
        swerveAngleFXConfig.Slot0.kV = Constants.Drivetrain.ANGLE_KF;
        swerveAngleFXConfig.MotorOutput.NeutralMode = Constants.Drivetrain.ANGLE_NEUTRAL_MODE;
        //swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;
        swerveAngleFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;


        /* Swerve Drive Motor Configuration */
        /*driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.Drivetrain.DRIVE_ENABLE_CURRENT_LIMIT, 
            Constants.Drivetrain.DRIVE_CONTINUOUS_CURRENT_LIMIT, 
            Constants.Drivetrain.DRIVE_PEAK_CURRENT_LIMIT, 
            Constants.Drivetrain.DRIVE_PEAK_CURRENT_DURATION);*/

            driveSupplyLimit.SupplyCurrentLimitEnable = Constants.Drivetrain.DRIVE_ENABLE_CURRENT_LIMIT;
            driveSupplyLimit.SupplyCurrentLimit = Constants.Drivetrain.DRIVE_CONTINUOUS_CURRENT_LIMIT;
            driveSupplyLimit.SupplyCurrentThreshold = Constants.Drivetrain.DRIVE_PEAK_CURRENT_LIMIT;
            driveSupplyLimit.SupplyTimeThreshold = Constants.Drivetrain.DRIVE_PEAK_CURRENT_DURATION;

        swerveDriveFXConfig.Slot0.kP = Constants.Drivetrain.DRIVE_KP;
        swerveDriveFXConfig.Slot0.kI = Constants.Drivetrain.DRIVE_KI;
        swerveDriveFXConfig.Slot0.kD = Constants.Drivetrain.DRIVE_KD;
        swerveDriveFXConfig.Slot0.kV = Constants.Drivetrain.DRIVE_KF;
        swerveDriveFXConfig.MotorOutput.NeutralMode = Constants.Drivetrain.DRIVE_NEUTRAL_MODE;        
        //swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        swerveDriveFXConfig.OpenLoopRamps = Constants.Drivetrain.OPEN_LOOP_RAMP;
        swerveDriveFXConfig.ClosedLoopRamps = Constants.Drivetrain.CLOSED_LOOP_RAMP;

        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = Constants.Drivetrain.CANCODER_INVERT;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

        

    }

}