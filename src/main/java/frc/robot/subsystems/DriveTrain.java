package frc.robot.subsystems;



import com.ctre.phoenix.sensors.Pigeon2;

import frc.lib.util.control.HolonomicMotionProfiledTrajectoryFollower;
import frc.lib.util.control.PidConstants;
import frc.lib.util.util.DrivetrainFeedforwardConstants;
import frc.lib.util.util.HolonomicFeedforward;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public GenericEntry mod0Cancoder, mod1Cancoder, mod2Cancoder, mod3Cancoder;
    private GenericEntry gyroHeading;

    public static final DrivetrainFeedforwardConstants FEEDFORWARD_CONSTANTS = new DrivetrainFeedforwardConstants(Constants.DriveTrain.DRIVE_KV,
    Constants.DriveTrain.DRIVE_KA, Constants.DriveTrain.DRIVE_KS);

    private final HolonomicMotionProfiledTrajectoryFollower follower = new HolonomicMotionProfiledTrajectoryFollower(
            new PidConstants(Constants.DriveTrain.DRIVE_KP, Constants.DriveTrain.DRIVE_KI, Constants.DriveTrain.DRIVE_KD), new PidConstants(Constants.DriveTrain.ANGLE_KP, Constants.DriveTrain.ANGLE_KI, Constants.DriveTrain.ANGLE_KD),
            new HolonomicFeedforward(FEEDFORWARD_CONSTANTS));

    public DriveTrain() {
        gyro = new Pigeon2(Constants.DriveTrain.PIGEON_ID, "CANivore");
        gyro.configFactoryDefault();
        zeroGyro();
        
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.DriveTrain.Mod0.CONSTANTS),
            new SwerveModule(1, Constants.DriveTrain.Mod1.CONSTANTS),
            new SwerveModule(2, Constants.DriveTrain.Mod2.CONSTANTS),
            new SwerveModule(3, Constants.DriveTrain.Mod3.CONSTANTS)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.DriveTrain.SWERVE_KINEMATICS, getYaw(), getStates());
        
       
        mod0Cancoder = Shuffleboard.getTab("Drive").add("Mod 0 Cancoder", mSwerveMods[0].getState().angle.getDegrees()).withPosition(0, 0).getEntry();
        mod1Cancoder = Shuffleboard.getTab("Drive").add("Mod 1 Cancoder", mSwerveMods[1].getState().angle.getDegrees()).withPosition(1, 0).getEntry();
        mod2Cancoder = Shuffleboard.getTab("Drive").add("Mod 2 Cancoder", mSwerveMods[2].getState().angle.getDegrees()).withPosition(2, 0).getEntry();
        mod3Cancoder = Shuffleboard.getTab("Drive").add("Mod 3 Cancoder", mSwerveMods[3].getState().angle.getDegrees()).withPosition(3, 0).getEntry();
        gyroHeading = Shuffleboard.getTab("Drive").add("Gyro", gyro.getYaw()).withPosition(0, 1).getEntry();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.DriveTrain.SWERVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.DriveTrain.MAX_SPEED);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }
    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.DriveTrain.MAX_SPEED);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public SwerveModulePosition[] getStates(){
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        return (Constants.DriveTrain.INVERT_GYRO) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getStates(), pose);
    }

    public HolonomicMotionProfiledTrajectoryFollower getFollower() {
        return follower;
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getStates());  
        
        mod0Cancoder.setDouble(mSwerveMods[0].getCanCoder().getDegrees());
        mod1Cancoder.setDouble(mSwerveMods[1].getCanCoder().getDegrees());
        mod2Cancoder.setDouble(mSwerveMods[2].getCanCoder().getDegrees());
        mod3Cancoder.setDouble(mSwerveMods[3].getCanCoder().getDegrees());
        
        gyroHeading.setDouble(gyro.getYaw());
    }   
}