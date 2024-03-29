package frc.robot;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import frc.lib.util.SwerveModuleConstants;

/* Name All Variables in ALL_CAPS Format */

public final class Constants {
    public static final boolean FIELD_RELATIVE = true;
    public static final boolean OPEN_LOOP = true;
    public static final int CANDLE_ID = 15;
    public static final String PRACTICE_MAC = "00:80:2F:25:DE:54";
    public static final boolean PRACTICE_BOT = getMACAddress().equals(PRACTICE_MAC);
    public static final boolean DEBUG = true;

    

    public static class VisionConstants {
        public static final Transform3d ROBOT_TO_CAM = new Transform3d(new Translation3d(0.3019665792, 0, 0.19), new Rotation3d(0, -0.349066, 0));
        public static final String CAMERA_NAME = "Global_Shutter_Camera";
        public static final double HIGH_LEFT_POST_X = 0.369;
        public static final double HIGH_LEFT_POST_Y = 0.513;
        public static final double HIGH_RIGHT_POST_X = 0.369;
        public static final double HIGH_RIGHT_POST_Y = 1.63;
        public static final double HIGH_MID_X = 0.369;
        public static final double HIGH_MID_Y = 1.07;
        public static final double HIGH_HEIGHT = 1.1684;

        public static final double MID_LEFT_POST_X = 0.803;
        public static final double MID_LEFT_POST_Y = 0.513;
        public static final double MID_RIGHT_POST_X = 0.803;
        public static final double MID_RIGHT_POST_Y = 1.63;
        public static final double MID_MID_X = 0.803;
        public static final double MID_MID_Y = 1.07;
        public static final double MID_HEIGHT = 0.8636;
    }

    public static final class Controllers{
        public static final GenericHID TRANSLATION_CONTROLLER = new GenericHID(0);
        public static final GenericHID ROTATION_CONTROLLER = new GenericHID(1);
        public static final GenericHID XBOX_CONTROLLER = new GenericHID(2);
        public static final GenericHID BUTTON_GRID = new GenericHID(3);
        public static final int XBOX_CONTROLLER_A_BUTTON = 1;
        public static final int XBOX_CONTROLLER_B_BUTTON = 2;
        public static final int XBOX_CONTROLLER_X_BUTTON = 3;
        public static final int XBOX_CONTROLLER_Y_BUTTON = 4;
        public static final int XBOX_CONTROLLER_LB_BUTTON = 5;
        public static final int XBOX_CONTROLLER_RB_BUTTON = 6;
        public static final int XBOX_CONTROLLER_BACK_BUTTON = 7;
        public static final int XBOX_CONTROLLER_START_BUTTON = 8;
        public static final int XBOX_CONTROLLER_LEFT_SIICK_BUTTON = 9;
        public static final int XBOX_CONTROLLER_RIGHT_STICK_BUTTON = 10;
        public static final int UP = 0;
        public static final int RIGHT = 90;
        public static final int DOWN = 180;
        public static final int LEFT = 270;
        public static final int BUTTON_GRID_HIGH_LEFT = 3;
        public static final int BUTTON_GRID_HIGH_MID = 2;
        public static final int BUTTON_GRID_HIGH_RIGHT = 1;
        public static final int BUTTON_GRID_MID_LEFT = 6;
        public static final int BUTTON_GRID_MID_MID = 5;
        public static final int BUTTON_GRID_MID_RIGHT = 4;
        public static final int BUTTON_GRID_HYBRID_LEFT = 7;
        public static final int BUTTON_GRID_HYBRID_MID = 8;
        public static final int BUTTON_GRID_HYBRID_RIGHT = 9;
        public static final double STICK_DEADBAND = 0.1;
        public static final int TRANSLATION_BUTTON = 1;
        public static final int ROTATION_BUTTON = 1;
        public static final int TRANSLATION_AXIS = 0;
        public static final int STRAFE_AXIS = 1;
        public static final int ROTATION_AXIS = 0;
    }

    public static final class Drivetrain {
        public static final int PIGEON_ID = 20;
        public static final boolean INVERT_GYRO = false;

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = 0.45;
        public static final double WHEEL_BASE = 0.645;
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.86);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        public static final double DRIVE_GEAR_RATIO = (6.75 / 1.0);
        public static final double ANGLE_GEAR_RATIO = (12.8 / 1.0);

        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

        /* Swerve Current Limiting */
        public static final int ANGLE_CONTINUOUS_CURRENT_LIMIT = 25;
        public static final int ANGLE_PEAK_CURRENT_LIMIT = 40;
        public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

        public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 35;
        public static final int DRIVE_PEAK_CURRENT_LIMIT = 50;
        public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        /* Angle Motor PID Values */
        public static final double ANGLE_KP = 26.3; 
        public static final double ANGLE_KI = 0.0;
        public static final double ANGLE_KD = 0.288;
        public static final double ANGLE_KF = 0.0;

        /* Drive Motor PID Values */
        public static final double DRIVE_KP = 0.024;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double DRIVE_KS = (0.53906 / 12);
        public static final double DRIVE_KV = (2.2756 / 12);
        public static final double DRIVE_KA = (0.065383 / 12);

        /* Swerve Profiling Values */
        public static final double MAX_SPEED = 4.5;
        public static final double MAX_ANGULAR_VELOCITY = 11.5;

        /* Neutral Modes */
        public static final NeutralModeValue ANGLE_NEUTRAL_MODE = NeutralModeValue.Brake;
        public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;

        /* Motor Inverts */
        public static final InvertedValue DRIVE_MOTOR_INVERT = InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue ANGLE_MOTOR_INVERT = InvertedValue.CounterClockwise_Positive;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue CANCODER_INVERT = SensorDirectionValue.Clockwise_Positive;

        /* Module Specific Constants */

        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int DRIVE_MOTOR_ID = 0;
            public static final int ANGLE_MOTOR_ID = 1;
            public static final int CANCODER_ID = 21;
            public static final double ANGLE_OFFSET = -0.7024;
            public static final double ANGLE_OFFSET_PRACTICE = 0;
            public static final SwerveModuleConstants CONSTANTS = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET, ANGLE_OFFSET_PRACTICE);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int DRIVE_MOTOR_ID = 19;
            public static final int ANGLE_MOTOR_ID = 18;
            public static final int CANCODER_ID = 22;
            public static final double ANGLE_OFFSET = -0.8889;
            public static final double ANGLE_OFFSET_PRACTICE = 0;
            public static final SwerveModuleConstants CONSTANTS = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET, ANGLE_OFFSET_PRACTICE);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int DRIVE_MOTOR_ID = 8;
            public static final int ANGLE_MOTOR_ID = 9;
            public static final int CANCODER_ID = 23;
            public static final double ANGLE_OFFSET = -0.8948;
            public static final double ANGLE_OFFSET_PRACTICE = 0;
            public static final SwerveModuleConstants CONSTANTS = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET, ANGLE_OFFSET_PRACTICE);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int DRIVE_MOTOR_ID = 11;
            public static final int ANGLE_MOTOR_ID = 10;
            public static final int CANCODER_ID = 24;
            public static final double ANGLE_OFFSET = -0.4138;
            public static final double ANGLE_OFFSET_PRACTICE = 0;
            public static final SwerveModuleConstants CONSTANTS = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET, ANGLE_OFFSET_PRACTICE);
        }

    }

    public static final class Auto {
        public static final double KMAX_SPEED_METERS_PER_SECOND = Integer.MAX_VALUE;
        public static final double KMAX_ACCELERATION_METERS_PER_SECOND_SQUARED = Integer.MAX_VALUE;
    
        public static final double KP_X_CONTROLLER = 1;
        public static final double KP_Y_CONTROLLER = 1;
        public static final double KP_THETA_CONTROLLER = 5;
    }

    public static final class Intake{
        public static final int INTAKE_MOTOR_ID = 6;
        public static final int WRIST_MOTOR_ID = 5; 
        public static final int US_SENSOR_ID = 0;
        public static final int BB_SENSOR_ID = 1;
    }

    public static final class Climber{
        public static final int LEAD_ID = 12;
        public static final int FOLLOWER_ID = 13;
    }

    public static final class Arm{
        public static final int LEAD_SHOULDER_ID = 3;
        public static final int FOLLOWER_SHOULDER_ID = 2;
        public static final int LEAD_EXTENSION_ID = 4;
        public static final int FOLLOWER_EXTENSION_ID = 16;
        public static final int TURRET_ID = 14;
    }

    public static String getMACAddress() {
        try{
            Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
            StringBuilder ret = new StringBuilder();
            while(nwInterface.hasMoreElements()){
                NetworkInterface nis = nwInterface.nextElement();
                if(nis != null && "eth0".equals(nis.getDisplayName())){
                    byte[] mac = nis.getHardwareAddress();
                    if(mac != null){
                        for(int i = 0; i < mac.length; i++){
                            ret.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? ":" : ""));
                        }
                        String addr = ret.toString();
                        return addr;
                    }
                    else {}
                } 
                else {}
            }
        } 
        catch (SocketException | NullPointerException e) {}
        return "";
    }

}
