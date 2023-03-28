/*
 * MIT License
 *
 * Copyright (c) 2022 PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.RunArm;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PhotonVision extends SubsystemBase{
    private final Intake s_Intake;
    private final LED s_LED;
    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator photonPoseEstimator;
    private final GenericEntry poseX, poseY, distance, angle;
    private AprilTagFieldLayout atfl;
    private boolean highLeft, highMid, highRight, midLeft, midMid, midRight, hybridLeft, hybridMid, hybridRight, auto = false;
    private ArrayList<Boolean> array = new ArrayList<Boolean>(9);
    private Pose2d pose2d = new Pose2d(0, 0, new Rotation2d(0));
    private Pose2d pose2dDrivetrain = new Pose2d(0, 0, new Rotation2d(0));
    private double x, y, targetX, targetY, targetHeight, theta, angleOffset;

    public PhotonVision(LED s_LED, Intake s_Intake) {
        this.s_LED = s_LED;
        this.s_Intake = s_Intake;
        try{
            atfl = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
        } 
        catch (IOException e) {}

        photonCamera = new PhotonCamera(VisionConstants.CAMERA_NAME);        
        photonPoseEstimator =
            new PhotonPoseEstimator(atfl, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, photonCamera, Constants.VisionConstants.ROBOT_TO_CAM);

        poseX = Shuffleboard.getTab("SyrupTag").add("Pose X", 0).withPosition(0, 0).getEntry();
        poseY = Shuffleboard.getTab("SyrupTag").add("Pose Y", 0).withPosition(1, 0).getEntry();
        distance = Shuffleboard.getTab("SyrupTag").add("Distance", 0).withPosition(2, 0).getEntry();
        angle = Shuffleboard.getTab("SyrupTag").add("Angle", 0).withPosition(3, 0).getEntry();
    }

    public Pose2d getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        Optional<EstimatedRobotPose> result = photonPoseEstimator.update();
        if(result.isPresent()){
            return result.get().estimatedPose.toPose2d();
        } 
        else{
            return prevEstimatedRobotPose;
        }
    }

    public void setHighLeft(){
        highLeft = true;
        highMid = false;
        highRight = false;
        midLeft = false;
        midMid = false;
        midRight = false;
        hybridLeft = false;
        hybridMid = false;
        hybridRight = false;
        if(DriverStation.getAlliance().toString().equals("Blue")){
            targetX = Constants.VisionConstants.HIGH_LEFT_POST_X;
            targetY = Constants.VisionConstants.HIGH_LEFT_POST_Y;
        }
        else{
            targetX = Constants.VisionConstants.HIGH_LEFT_POST_X;
            targetY = Constants.VisionConstants.HIGH_LEFT_POST_Y + 1.1;
        }
        targetHeight = Constants.VisionConstants.HIGH_HEIGHT;
    }
    
    public void setHighRight(){
        highLeft = false;
        highMid = false;
        highRight = true;
        midLeft = false;
        midMid = false;
        midRight = false;
        hybridLeft = false;
        hybridMid = false;
        hybridRight = false;
        if(DriverStation.getAlliance().toString().equals("Blue")){
            targetX = Constants.VisionConstants.HIGH_RIGHT_POST_X;
            targetY = Constants.VisionConstants.HIGH_RIGHT_POST_Y;
        }
        else{
            targetX = Constants.VisionConstants.HIGH_RIGHT_POST_X;
            targetY = Constants.VisionConstants.HIGH_RIGHT_POST_Y - 1.1;
        }
        targetHeight = Constants.VisionConstants.HIGH_HEIGHT;
    }

    public void setHighMid(){
        highLeft = false;
        highMid = true;
        highRight = false;
        midLeft = false;
        midMid = false;
        midRight = false;
        hybridLeft = false;
        hybridMid = false;
        hybridRight = false;
        targetX = Constants.VisionConstants.HIGH_MID_X;
        targetY = Constants.VisionConstants.HIGH_MID_Y;
        targetHeight = Constants.VisionConstants.HIGH_HEIGHT;
    }

    public void setMidLeft(){
        highLeft = false;
        highMid = false;
        highRight = false;
        midLeft = true;
        midMid = false;
        midRight = false;
        hybridLeft = false;
        hybridMid = false;
        hybridRight = false;
        if(DriverStation.getAlliance().toString().equals("Blue")){
            targetX = Constants.VisionConstants.MID_LEFT_POST_X;
            targetY = Constants.VisionConstants.MID_LEFT_POST_Y;
        }
        else{
            targetX = Constants.VisionConstants.MID_LEFT_POST_X;
            targetY = Constants.VisionConstants.MID_LEFT_POST_Y + 1.1;
        }
        targetHeight = Constants.VisionConstants.MID_HEIGHT;
    }

    public void setMidRight(){
        highLeft = false;
        highMid = false;
        highRight = false;
        midLeft = false;
        midMid = false;
        midRight = true;
        hybridLeft = false;
        hybridMid = false;
        hybridRight = false;
        if(DriverStation.getAlliance().toString().equals("Blue")){
            targetX = Constants.VisionConstants.MID_RIGHT_POST_X;
            targetY = Constants.VisionConstants.MID_RIGHT_POST_Y;
        }
        else{
            targetX = Constants.VisionConstants.MID_RIGHT_POST_X;
            targetY = Constants.VisionConstants.MID_RIGHT_POST_Y - 1.1;
        }
        targetHeight = Constants.VisionConstants.MID_HEIGHT;
    }

    public void setMidMid(){
        highLeft = false;
        highMid = false;
        highRight = false;
        midLeft = false;
        midMid = true;
        midRight = false;
        hybridLeft = false;
        hybridMid = false;
        hybridRight = false;
        targetX = Constants.VisionConstants.MID_MID_X;
        targetY = Constants.VisionConstants.MID_MID_Y;
        targetHeight = Constants.VisionConstants.MID_HEIGHT;
    }

    public void setHybridMid(){
        highLeft = false;
        highMid = false;
        highRight = false;
        midLeft = false;
        midMid = false;
        midRight = false;
        hybridLeft = false;
        hybridMid = true;
        hybridRight = false;
    }

    public void setHybridLeft(){
        highLeft = false;
        highMid = false;
        highRight = false;
        midLeft = false;
        midMid = false;
        midRight = false;
        hybridLeft = true;
        hybridMid = false;
        hybridRight = false;
    }

    public void setHybridRight(){
        highLeft = false;
        highMid = false;
        highRight = false;
        midLeft = false;
        midMid = false;
        midRight = false;
        hybridLeft = false;
        hybridMid = false;
        hybridRight = true;
    }

    public ArrayList<Boolean> getSelectedScore(){
        array.clear();
        array.add(0, highLeft); 
        array.add(1, highMid);
        array.add(2, highRight);
        array.add(3, midLeft);
        array.add(4, midMid);
        array.add(5, midRight); 
        array.add(6, hybridLeft); 
        array.add(7, hybridMid);
        array.add(8, hybridRight);
        return array;
    }

    public void setAuto(){
        if(!auto){
            auto = true;
        }
        else{
            auto = false;
        }
    }

    public void setAllFalse(){
        highLeft = false;
        highMid = false;
        highRight = false;
        midLeft = false;
        midMid = false;
        midRight = false;
        hybridLeft = false;
        hybridMid = false;
        hybridRight = false;
    }

    public boolean getAuto(){
        return auto;
    }

    public double getAutoTrackDistance(){
        pose2d = getEstimatedGlobalPose(pose2dDrivetrain);
        if(!pose2d.equals(Robot.m_robotContainer.s_Drivetrain.getPose()) && photonCamera.getLatestResult().getBestTarget() != null){
            Robot.m_robotContainer.s_Drivetrain.resetOdometry(pose2d);
            pose2dDrivetrain = pose2d;
        }
        else{
            pose2dDrivetrain = Robot.m_robotContainer.s_Drivetrain.getPose();
        }
        if(DriverStation.getAlliance().toString().equals("Blue")){
            if(pose2dDrivetrain.getY() > -1 && pose2dDrivetrain.getY() < 1.9 && ((pose2dDrivetrain.getY() < 1.05 && pose2dDrivetrain.getY() - 0.15 > -1) || (pose2dDrivetrain.getY() > 1.05 && pose2dDrivetrain.getY() + 0.15 < 1.9))){
                x = targetX - pose2dDrivetrain.getX();
                y = targetY - pose2dDrivetrain.getY();
            }
            else if(pose2dDrivetrain.getY() > 1.9 && pose2dDrivetrain.getY() < 3.6 && ((pose2dDrivetrain.getY() < 1.05 + 1.7 && pose2dDrivetrain.getY() - 0.15 > 1.9) || (pose2dDrivetrain.getY() > 1.05 + 1.7 && pose2dDrivetrain.getY() + 0.15 < 3.6))){
                x = targetX - pose2dDrivetrain.getX();
                y = (targetY + 1.7) - pose2dDrivetrain.getY();
            }
            else if(pose2dDrivetrain.getY() > 3.6 && pose2dDrivetrain.getY() < 6 && ((pose2dDrivetrain.getY() < 1.05 + 1.7 + 1.7 && pose2dDrivetrain.getY() - 0.15 > 3.6) || (pose2dDrivetrain.getY() > 1.05 + 1.7 + 1.7 && pose2dDrivetrain.getY() + 0.15 < 6))){
                x = targetX - pose2dDrivetrain.getX();
                y = (targetY + 1.7 + 1.7) - pose2dDrivetrain.getY();
            }
        }
        else{
            if(pose2dDrivetrain.getY() > -1 && pose2dDrivetrain.getY() < 1.9 && ((pose2dDrivetrain.getY() < 1.05 && pose2dDrivetrain.getY() - 0.15 > -1) || (pose2dDrivetrain.getY() > 1.05 && pose2dDrivetrain.getY() + 0.15 < 1.9))){
                x = targetX - 16.5 - pose2dDrivetrain.getX();
                y = targetY - pose2dDrivetrain.getY();
            }
            else if(pose2dDrivetrain.getY() > 1.9 && pose2dDrivetrain.getY() < 3.6 && ((pose2dDrivetrain.getY() < 1.05 + 1.7 && pose2dDrivetrain.getY() - 0.15 > 1.9) || (pose2dDrivetrain.getY() > 1.05 + 1.7 && pose2dDrivetrain.getY() + 0.15 < 3.6))){
                x = targetX - 16.5 - pose2dDrivetrain.getX();
                y = (targetY + 1.7) - pose2dDrivetrain.getY();
            }
            else if(pose2dDrivetrain.getY() > 3.6 && pose2dDrivetrain.getY() < 6 && ((pose2dDrivetrain.getY() < 1.05 + 1.7 + 1.7 && pose2dDrivetrain.getY() - 0.15 > 3.6) || (pose2dDrivetrain.getY() > 1.05 + 1.7 + 1.7 && pose2dDrivetrain.getY() + 0.15 < 6))){
                x = targetX - 16.5 - pose2dDrivetrain.getX();
                y = (targetY + 1.7 + 1.7) - pose2dDrivetrain.getY();
            }
        }
        return (Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(targetHeight - 0.48, 2)) - 0.61) * -50000;
    }

    public double getAutoTrackAngle(){
        if(s_Intake.getDistance() > 0.45){
            angleOffset = 0;
        }
        else{
            angleOffset = Math.toDegrees(Math.atan((s_Intake.getDistance() + 0.05 - 0.27)/(RunArm.getExtensionValue() / -50000 + 0.61)));
        }
        if(y == 0 && s_Intake.getDistance() > 0.45){
            return 0;
        }
        else if(y == 0 && s_Intake.getDistance() < 0.45){
            return angleOffset / 5.78;
        }
        theta = -Math.atan(y / x);
        theta = Math.toDegrees(theta);
        theta = 0;
        theta = (180 - (Robot.m_robotContainer.s_Drivetrain.getYaw().getDegrees() + 3600000) % 360) - theta;
        return (theta - angleOffset) / 5.78;
    }

    @Override
    public void periodic(){
        if(DriverStation.isTeleopEnabled()){
            distance.setDouble(getAutoTrackDistance());
            angle.setDouble(getAutoTrackAngle());
            poseX.setDouble(pose2dDrivetrain.getX());
            poseY.setDouble(pose2dDrivetrain.getY());
        }
        if(auto){
            if(!photonCamera.isConnected()){
                s_LED.bad();
            }
            if(photonCamera.isConnected()){
                s_LED.notBad();
            }
            if(DriverStation.getAlliance().toString().equals("Blue") && pose2dDrivetrain.getX() > 4){
                s_LED.red();
            }
            else if(DriverStation.getAlliance().toString().equals("Red") && 16.5 - pose2dDrivetrain.getX() > 4){
                s_LED.red();
            }
            else if(getAutoTrackAngle() < -5 || getAutoTrackAngle() > 5){
                s_LED.white();
            }
            else if(getAutoTrackAngle() >= -5 && getAutoTrackAngle() <= 5){
                s_LED.green();
            }
        }
        else if(!auto){
            s_LED.setTrackingLEDsOff(!photonCamera.isConnected());
        }
    }
}