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

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PhotonVision extends SubsystemBase{
    private final LED s_LED;
    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator photonPoseEstimator;
    private final GenericEntry poseX, poseY, distance, angle;
    private AprilTagFieldLayout atfl;
    private boolean highLeft, highMid, highRight, midLeft, midMid, midRight, hybridLeft, hybridMid, hybridRight, auto = false;
    private ArrayList<Boolean> array = new ArrayList<Boolean>(9);
    private Pose2d pose2d = new Pose2d(0, 0, new Rotation2d(0));
    private Pose2d pose2dDrivetrain = new Pose2d(0, 0, new Rotation2d(0));
    private double x, y, targetX, targetY, theta;

    public PhotonVision(LED s_LED) {
        this.s_LED = s_LED;
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
        targetX = Constants.VisionConstants.MID_HYBRID_X;
        targetY = Constants.VisionConstants.MID_HYBRID_Y;
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
        if(DriverStation.getAlliance().toString().equals("Blue")){
            targetX = Constants.VisionConstants.LEFT_HYBRID_X;
            targetY = Constants.VisionConstants.LEFT_HYBRID_Y;
        }
        else{
            targetX = Constants.VisionConstants.LEFT_HYBRID_X;
            targetY = Constants.VisionConstants.LEFT_HYBRID_Y + 1.1;
        }
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
        if(DriverStation.getAlliance().toString().equals("Blue")){
            targetX = Constants.VisionConstants.RIGHT_HYBRID_X;
            targetY = Constants.VisionConstants.RIGHT_HYBRID_Y;
        }
        else{
            targetX = Constants.VisionConstants.RIGHT_HYBRID_X;
            targetY = Constants.VisionConstants.RIGHT_HYBRID_Y - 1.1;
        }
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
        if(!getEstimatedGlobalPose(pose2d).equals(Robot.m_robotContainer.s_Drivetrain.getPose()) && photonCamera.getLatestResult().getBestTarget() != null){
            Robot.m_robotContainer.s_Drivetrain.resetOdometry(pose2d);
            pose2dDrivetrain = pose2d;
        }
        else{
            pose2dDrivetrain = Robot.m_robotContainer.s_Drivetrain.getPose();
        }
        if(DriverStation.getAlliance().toString().equals("Blue")){
            if(pose2dDrivetrain.getY() > -1 && pose2dDrivetrain.getY() < 1.9 && ((pose2dDrivetrain.getY() < 1.05 && pose2dDrivetrain.getY() - 0.15 > -1) || (pose2dDrivetrain.getY() > 1.05 && pose2dDrivetrain.getY() + 0.15 < 1.9))){
                x = pose2dDrivetrain.getX() - targetX;
                y = pose2dDrivetrain.getY() - targetY;
            }
            else if(pose2dDrivetrain.getY() > 1.9 && pose2dDrivetrain.getY() < 3.6 && ((pose2dDrivetrain.getY() < 1.05 + 1.7 && pose2dDrivetrain.getY() - 0.15 > 1.9) || (pose2dDrivetrain.getY() > 1.05 + 1.7 && pose2dDrivetrain.getY() + 0.15 < 3.6))){
                x = pose2dDrivetrain.getX() - targetX;
                y = pose2dDrivetrain.getY() - (targetY + 1.7);
            }
            else if(pose2dDrivetrain.getY() > 3.6 && pose2dDrivetrain.getY() < 6 && ((pose2dDrivetrain.getY() < 1.05 + 1.7 + 1.7 && pose2dDrivetrain.getY() - 0.15 > 3.6) || (pose2dDrivetrain.getY() > 1.05 + 1.7 + 1.7 && pose2dDrivetrain.getY() + 0.15 < 6))){
                x = pose2dDrivetrain.getX() - targetX;
                y = pose2dDrivetrain.getY() - (targetY + 1.7 + 1.7);
            }
        }
        else{
            if(pose2dDrivetrain.getY() > -1 && pose2dDrivetrain.getY() < 1.9 && ((pose2dDrivetrain.getY() < 1.05 && pose2dDrivetrain.getY() - 0.15 > -1) || (pose2dDrivetrain.getY() > 1.05 && pose2dDrivetrain.getY() + 0.15 < 1.9))){
                x = 16.5 - pose2dDrivetrain.getX() - targetX;
                y = pose2dDrivetrain.getY() - targetY;
            }
            else if(pose2dDrivetrain.getY() > 1.9 && pose2dDrivetrain.getY() < 3.6 && ((pose2dDrivetrain.getY() < 1.05 + 1.7 && pose2dDrivetrain.getY() - 0.15 > 1.9) || (pose2dDrivetrain.getY() > 1.05 + 1.7 && pose2dDrivetrain.getY() + 0.15 < 3.6))){
                x = 16.5 - pose2dDrivetrain.getX() - targetX;
                y = pose2dDrivetrain.getY() - (targetY + 1.7);
            }
            else if(pose2dDrivetrain.getY() > 3.6 && pose2dDrivetrain.getY() < 6 && ((pose2dDrivetrain.getY() < 1.05 + 1.7 + 1.7 && pose2dDrivetrain.getY() - 0.15 > 3.6) || (pose2dDrivetrain.getY() > 1.05 + 1.7 + 1.7 && pose2dDrivetrain.getY() + 0.15 < 6))){
                x = 16.5 - pose2dDrivetrain.getX() - targetX;
                y = pose2dDrivetrain.getY() - (targetY + 1.7 + 1.7);
            }
        }
        return (Math.sqrt(x * x + y * y) - 0.35) * -50000;
    }

    public double getAutoTrackAngle(){
        if(y == 0){
            return 0;
        }
        else{
            theta = Math.atan(x / y);
            theta = Robot.m_robotContainer.s_Drivetrain.getYaw().getRadians() - theta - Math.PI / 2;
            theta = Math.toDegrees(theta);
            return theta / 5.78;
        }
    }

    @Override
    public void periodic(){
        distance.setDouble(getAutoTrackDistance());
        angle.setDouble(getAutoTrackAngle());
        poseX.setDouble(pose2dDrivetrain.getX());
        poseY.setDouble(pose2dDrivetrain.getY());
        if(auto){
            if(!photonCamera.isConnected()){
                s_LED.bad();
            }
            else if(getAutoTrackDistance() >= -46500 && getAutoTrackAngle() >= -5 && getAutoTrackAngle() <= 5){
                s_LED.green();
            }
            else if(getAutoTrackDistance() < -46500 || getAutoTrackAngle() < -5 || getAutoTrackAngle() > 5){
                s_LED.white();
            }
            else if(pose2dDrivetrain.getX() > 4){
                s_LED.red();
            }
        }
        else if(!auto){
            s_LED.notBad();
        }
    }
}