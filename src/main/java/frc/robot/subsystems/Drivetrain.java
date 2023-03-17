package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

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

public class Drivetrain extends SubsystemBase {
    private final SwerveDriveOdometry swerveOdometry;
    private final SwerveModule[] swerveMods;
    private final Pigeon2 gyro;
    private GenericEntry mod0Cancoder, mod1Cancoder, mod2Cancoder, mod3Cancoder;
    private GenericEntry yaw, roll;

    public Drivetrain() {
        gyro = new Pigeon2(Constants.Drivetrain.PIGEON_ID, "CANivore");
        gyro.configFactoryDefault();
        zeroGyro();
        
        swerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Drivetrain.Mod0.CONSTANTS),
            new SwerveModule(1, Constants.Drivetrain.Mod1.CONSTANTS),
            new SwerveModule(2, Constants.Drivetrain.Mod2.CONSTANTS),
            new SwerveModule(3, Constants.Drivetrain.Mod3.CONSTANTS)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Drivetrain.SWERVE_KINEMATICS, getYaw(), getPositions());
        
       
        mod0Cancoder = Shuffleboard.getTab("Drive").add("Mod 0 Cancoder", swerveMods[0].getState().angle.getDegrees()).withPosition(0, 0).getEntry();
        mod1Cancoder = Shuffleboard.getTab("Drive").add("Mod 1 Cancoder", swerveMods[1].getState().angle.getDegrees()).withPosition(1, 0).getEntry();
        mod2Cancoder = Shuffleboard.getTab("Drive").add("Mod 2 Cancoder", swerveMods[2].getState().angle.getDegrees()).withPosition(2, 0).getEntry();
        mod3Cancoder = Shuffleboard.getTab("Drive").add("Mod 3 Cancoder", swerveMods[3].getState().angle.getDegrees()).withPosition(3, 0).getEntry();
        yaw = Shuffleboard.getTab("Drive").add("Yaw", gyro.getYaw()).withPosition(0, 1).getEntry();
        roll = Shuffleboard.getTab("Drive").add("Roll", gyro.getRoll()).withPosition(1, 1).getEntry();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Drivetrain.SWERVE_KINEMATICS.toSwerveModuleStates(
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
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Drivetrain.MAX_SPEED);

        for(SwerveModule mod : swerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : swerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getPositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : swerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.setYaw(180);
    }

    public Rotation2d getYaw() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        return (Constants.Drivetrain.INVERT_GYRO) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
    }

    public double getRoll(){
        return gyro.getRoll();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Drivetrain.MAX_SPEED);
        
        for(SwerveModule mod : swerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    } 

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getPositions());  
        
        mod0Cancoder.setDouble(swerveMods[0].getCanCoder().getDegrees());
        mod1Cancoder.setDouble(swerveMods[1].getCanCoder().getDegrees());
        mod2Cancoder.setDouble(swerveMods[2].getCanCoder().getDegrees());
        mod3Cancoder.setDouble(swerveMods[3].getCanCoder().getDegrees());
        
        yaw.setDouble(gyro.getYaw());
        roll.setDouble(gyro.getRoll());
    }   
}