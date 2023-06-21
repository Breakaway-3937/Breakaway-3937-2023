package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;

    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();

        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Drivetrain.ANGLE_ENABLE_CURRENT_LIMIT;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Drivetrain.ANGLE_CONTINUOUS_CURRENT_LIMIT;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.Drivetrain.ANGLE_PEAK_CURRENT_LIMIT;
        swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.Drivetrain.ANGLE_PEAK_CURRENT_DURATION;

        swerveAngleFXConfig.Slot0.kP = Constants.Drivetrain.ANGLE_KP;
        swerveAngleFXConfig.Slot0.kI = Constants.Drivetrain.ANGLE_KI;
        swerveAngleFXConfig.Slot0.kD = Constants.Drivetrain.ANGLE_KD;
        swerveAngleFXConfig.Slot0.kV = Constants.Drivetrain.ANGLE_KF;
        swerveAngleFXConfig.MotorOutput.NeutralMode = Constants.Drivetrain.ANGLE_NEUTRAL_MODE;
        swerveAngleFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        swerveAngleFXConfig.Feedback.RotorToSensorRatio = Constants.Drivetrain.ANGLE_GEAR_RATIO;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        swerveAngleFXConfig.MotorOutput.Inverted = Constants.Drivetrain.ANGLE_MOTOR_INVERT;


        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Drivetrain.DRIVE_ENABLE_CURRENT_LIMIT;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Drivetrain.DRIVE_CONTINUOUS_CURRENT_LIMIT;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.Drivetrain.DRIVE_PEAK_CURRENT_LIMIT;
        swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.Drivetrain.DRIVE_PEAK_CURRENT_DURATION;

        swerveDriveFXConfig.Slot0.kP = Constants.Drivetrain.DRIVE_KP;
        swerveDriveFXConfig.Slot0.kI = Constants.Drivetrain.DRIVE_KI;
        swerveDriveFXConfig.Slot0.kD = Constants.Drivetrain.DRIVE_KD;
        swerveDriveFXConfig.Slot0.kV = Constants.Drivetrain.DRIVE_KF;
        swerveDriveFXConfig.MotorOutput.NeutralMode = Constants.Drivetrain.DRIVE_NEUTRAL_MODE;        
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Drivetrain.OPEN_LOOP_RAMP;
        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Drivetrain.CLOSED_LOOP_RAMP;
    }

}