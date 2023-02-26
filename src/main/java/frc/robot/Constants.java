package frc.robot;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.SparkMaxAlternateEncoder;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

    

    public static class VisionConstants {
        public static final Transform3d ROBOT_TO_CAM =
                new Transform3d(
                        new Translation3d(-0.15, -.10, 0.36),
                        new Rotation3d(0, 0, 0));
        public static final String CAMERA_NAME = "Global_Shutter_Camera";
        public static final double HIGH_LEFT_POST_X = -0.65;
        public static final double HIGH_LEFT_POST_Y = 0.560;
        public static final double HIGH_RIGHT_POST_X = -0.65;
        public static final double HIGH_RIGHT_POST_Y = -0.560;
        public static final double HIGH_MID_X = -0.65;
        public static final double HIGH_MID_Y = 0.0;
        public static final double MID_MID_X = -0.22;
        public static final double MID_MID_Y = 0.0;
        public static final double MID_LEFT_POST_X = -0.220;
        public static final double MID_LEFT_POST_Y = 0.560;
        public static final double MID_RIGHT_POST_X = -0.220;
        public static final double MID_RIGHT_POST_Y = -0.560;
        public static final double HIGH_DISTANCE = 0.85796;
        public static final double MID_DISTANCE = 0.60166;
        public static final double HIGH_MID_DISTANCE = 0.65;
        public static final double MID_MID_DISTANCE = 0.22;
        public static final double MAX_EXTEND_LENGTH = 1.22;
        public static final double LEFT_HYBRID_X = 0.177673;
        public static final double LEFT_HYBRID_Y = 0.560;
        public static final double RIGHT_HYBRID_X = 0.177673;
        public static final double RIGHT_HYBRID_Y = -0.560;
        public static final double HYBRID_DISTANCE = 0.58750974027;
        public static final double MID_HYBRID_X = 0.177673;
        public static final double MID_HYBRID_Y = 0.0;
        public static final double MID_HYBRID_DISTANCE = 0.177673;
    }

    public static final class Controllers{
        public static final GenericHID TRANSLATION_CONTROLLER = new GenericHID(0);
        public static final GenericHID ROTATION_CONTROLLER = new GenericHID(1);
        public static final GenericHID XBOX_CONTROLLER = new GenericHID(2);
        public static final int XBOXCONTROLLER_A_BUTTON = 1;
        public static final int XBOXCONTROLLER_B_BUTTON = 2;
        public static final int XBOXCONTROLLER_X_BUTTON = 3;
        public static final int XBOXCONTROLLER_Y_BUTTON = 4;
        public static final int XBOXCONTROLLER_LB_BUTTON = 5;
        public static final int XBOXCONTROLLER_RB_BUTTON = 6;
        public static final int XBOXCONTROLLER_BACK_BUTTON = 7;
        public static final int XBOXCONTROLLER_START_BUTTON = 8;
        public static final int XBOXCONTROLLER_LEFT_SITCK_BUTTON = 9;
        public static final int XBOXCONTROLLER_RIGHT_SITCK_BUTTON = 10;
        public static final double STICK_DEADBAND = 0.1;
        public static final int TRANSLATION_BUTTON = 1;
        public static final int ROTATION_BUTTON = 1;
        public static final int TRANSLATION_AXIS = 0;
        public static final int STRAFE_AXIS = 1;
        public static final int ROTATION_AXIS = 0;
    }

    public static final class DriveTrain {
        public static final int PIGEON_ID = 20;
        public static final boolean INVERT_GYRO = false;

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = 0.45;
        public static final double WHEEL_BASE = 0.645;
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.925);
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
        public static final int DRIVE_PEAK_CURRENT_LIMIT = 60;
        public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        /* Angle Motor PID Values */
        public static final double ANGLE_KP = 0.7;
        public static final double ANGLE_KI = 0.0;
        public static final double ANGLE_KD = 12.0;
        public static final double ANGLE_KF = 0.0;

        /* Drive Motor PID Values */
        public static final double DRIVE_KP = 0.12;
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
        public static final NeutralMode ANGLE_NEUTRAL_MODE = NeutralMode.Brake;
        public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Coast;

        /* Motor Inverts */
        public static final boolean DRIVE_MOTOR_INVERT = false;
        public static final boolean ANGLE_MOTOR_INVERT = false;

        /* Angle Encoder Invert */
        public static final boolean CANCODER_INVERT = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int DRIVE_MOTOR_ID = 0;
            public static final int ANGLE_MOTOR_ID = 1;
            public static final int CANCODER_ID = 21;
            
            public static final double ANGLE_OFFSET = 72.6855 + 180.0;
            public static final double ANGLE_OFFSET_PRACTICE = 60.4685;
            public static final SwerveModuleConstants CONSTANTS = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET, ANGLE_OFFSET_PRACTICE);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int DRIVE_MOTOR_ID = 19;
            public static final int ANGLE_MOTOR_ID = 18;
            public static final int CANCODER_ID = 22;
            public static final double ANGLE_OFFSET = 138.6035 + 180.0;
            public static final double ANGLE_OFFSET_PRACTICE = 202.9 + 180.0;
            public static final SwerveModuleConstants CONSTANTS = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET, ANGLE_OFFSET_PRACTICE);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int DRIVE_MOTOR_ID = 8;
            public static final int ANGLE_MOTOR_ID = 9;
            public static final int CANCODER_ID = 23;
            public static final double ANGLE_OFFSET = 143.1738 + 180.0;
            public static final double ANGLE_OFFSET_PRACTICE = 7.4 + 180.0;
            public static final SwerveModuleConstants CONSTANTS = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET, ANGLE_OFFSET_PRACTICE);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int DRIVE_MOTOR_ID = 11;
            public static final int ANGLE_MOTOR_ID = 10;
            public static final int CANCODER_ID = 24;
            public static final double ANGLE_OFFSET = 330.9960 + 180.0;
            public static final double ANGLE_OFFSET_PRACTICE = 181.2304 + 180.0;
            public static final SwerveModuleConstants CONSTANTS = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET, ANGLE_OFFSET_PRACTICE);
        }

    }

    public static final class Auto {
        public static final double KMAX_SPEED_METERS_PER_SECOND = 3;
        public static final double KMAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
        public static final double KMAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
        public static final double KMAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;
    
        public static final double KP_X_CONTROLLER = 1;
        public static final double KP_Y_CONTROLLER = 1;
        public static final double KP_THETA_CONTROLLER = 5;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints KTHETA_CONTROLLER_CONSTRAINTS =
            new TrapezoidProfile.Constraints(
                KMAX_ANGULAR_SPEED_RADIANS_PER_SECOND, KMAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);
    }

    public static final class Intake{
        public static final int INTAKE_MOTOR_ID = 6;
        public static final int WRIST_MOTOR_ID = 5; 
        public static final int US_SENSOR_ID = 0;
        public static final int LIGHT_SENSOR_ID = 1;
    }

    public static final class Climber{
        public static final int ID_1 = 12;
        public static final int ID_2 = 13;
    }

    public static final class Arm{
        public static final int SHOULDER_ID = 3;
        public static final int SHOULDER_2_ID = 2;
        public static final int EXTENSION_ID = 4;
        public static final int EXTENSION_ID_1 = 16;
        public static final int ROTATION_ID = 14;
        public static final SparkMaxAlternateEncoder.Type ALT_ENC_TYPE = SparkMaxAlternateEncoder.Type.kQuadrature;
        public static final int CPR = 8192;
    }

    public static String getMACAddress() {
        try {
            Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
            StringBuilder ret = new StringBuilder();
            while (nwInterface.hasMoreElements()) {
                NetworkInterface nis = nwInterface.nextElement();
                System.out.println("NIS: " + nis.getDisplayName());
                if (nis != null && "eth0".equals(nis.getDisplayName())) {
                    byte[] mac = nis.getHardwareAddress();
                    if (mac != null) {
                        for (int i = 0; i < mac.length; i++) {
                            ret.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? ":" : ""));
                        }
                        String addr = ret.toString();
                        System.out.println("NIS " + nis.getDisplayName() + " addr: " + addr);
                        return addr;
                    } else {
                        System.out.println("Address doesn't exist or is not accessible");
                    }
                } else {
                    System.out.println("Skipping adaptor: " + nis.getDisplayName());
                }
            }
        } catch (SocketException | NullPointerException e) {
            e.printStackTrace();
        }

        return "";
    }

}
