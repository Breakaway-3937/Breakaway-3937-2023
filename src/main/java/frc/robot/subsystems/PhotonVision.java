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
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
    private AprilTagFieldLayout atfl;
    private GenericEntry distanceBoard, angleBoard;
    private double x, y, d, a;
    private double rY, rX, rR, dP, pR, theta, pX, pY, cos, angle;
    private boolean highLeft, highMid, highRight, midLeft, midMid, midRight, hybridLeft, hybridMid, hybridRight = false;
    private ArrayList<Boolean> array = new ArrayList<Boolean>(9);

    public PhotonVision(LED s_LED) {
        this.s_LED = s_LED;
        distanceBoard = Shuffleboard.getTab("SyrupTag").add("Distance", d).withPosition(0, 0).getEntry();
        angleBoard = Shuffleboard.getTab("SyrupTag").add("Angle", a).withPosition(1, 0).getEntry();                
        try {
            atfl = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
        } catch (IOException e) {}

        photonCamera =
                new PhotonCamera(
                        VisionConstants
                                .CAMERA_NAME);

        var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
        camList.add(new Pair<PhotonCamera, Transform3d>(photonCamera, VisionConstants.ROBOT_TO_CAM));
        
        photonPoseEstimator =
                new PhotonPoseEstimator(atfl, PoseStrategy.AVERAGE_BEST_TARGETS, photonCamera, camList.get(0).getSecond());
    }

    /**
     * @param estimatedRobotPose The current best guess at robot PoseX
     * @return A pair of the fused camera observations to a single Pose2d on the field, and the time
     *     of the observation. Assumes a planar field and the robot is always firmly on the ground
     */
    public Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);

        double currentTime = Timer.getFPGATimestamp();
        Optional<EstimatedRobotPose> result = photonPoseEstimator.update();
        if (result.isPresent()) {
            return new Pair<Pose2d, Double>(
                    result.get().estimatedPose.toPose2d(), currentTime - result.get().timestampSeconds);
        } else {
            return new Pair<Pose2d, Double>(prevEstimatedRobotPose, 0.0);
        }
    }


    public Pair<Double, Double> getArmStuff(){
        var result = photonCamera.getLatestResult();
        if(result.hasTargets()){
            rY = result.getBestTarget().getBestCameraToTarget().getY();
            rX = result.getBestTarget().getBestCameraToTarget().getX();
            angle = result.getBestTarget().getBestCameraToTarget().getRotation().getAngle();
            rR = Math.sqrt(Math.pow(rX, 2) + Math.pow(rY, 2));
            dP = Math.sqrt(Math.pow((pX - rX), 2) + Math.pow((pY - rY), 2));
            cos = Math.abs((Math.pow(pR, 2) - Math.pow(rR, 2) - Math.pow(dP, 2)) / (2 * (rR * dP)));
            theta = Math.toDegrees(Math.acos(cos));
            return new Pair<Double, Double>(dP, theta);
        }
        else{
            return new Pair<Double,Double>(0.0, 0.0);
        }
    }

    public void setHighLeft(){
        pX = Constants.VisionConstants.HIGH_LEFT_POST_X;
        pY = Constants.VisionConstants.HIGH_LEFT_POST_Y;
        pR = Constants.VisionConstants.HIGH_DISTANCE;
        highLeft = true;
        highMid = false;
        highRight = false;
        midLeft = false;
        midMid = false;
        midRight = false;
        hybridLeft = false;
        hybridMid = false;
        hybridRight = false;
    }
    
    public void setHighRight(){
        pX = Constants.VisionConstants.HIGH_RIGHT_POST_X;
        pY = Constants.VisionConstants.HIGH_RIGHT_POST_Y;
        pR = Constants.VisionConstants.HIGH_DISTANCE;
        highLeft = false;
        highMid = false;
        highRight = true;
        midLeft = false;
        midMid = false;
        midRight = false;
        hybridLeft = false;
        hybridMid = false;
        hybridRight = false;
    }

    public void setHighMid(){
        pX = Constants.VisionConstants.HIGH_MID_X;
        pY = Constants.VisionConstants.HIGH_MID_Y;
        pR = Constants.VisionConstants.HIGH_MID_DISTANCE;
        highLeft = false;
        highMid = true;
        highRight = false;
        midLeft = false;
        midMid = false;
        midRight = false;
        hybridLeft = false;
        hybridMid = false;
        hybridRight = false;
    }

    public void setMidLeft(){
        pX = Constants.VisionConstants.MID_LEFT_POST_X;
        pY = Constants.VisionConstants.MID_LEFT_POST_Y;
        pR = Constants.VisionConstants.MID_DISTANCE;
        highLeft = false;
        highMid = false;
        highRight = false;
        midLeft = true;
        midMid = false;
        midRight = false;
        hybridLeft = false;
        hybridMid = false;
        hybridRight = false;
    }

    public void setMidRight(){
        pX = Constants.VisionConstants.MID_RIGHT_POST_X;
        pY = Constants.VisionConstants.MID_RIGHT_POST_Y;
        pR = Constants.VisionConstants.MID_DISTANCE;
        highLeft = false;
        highMid = false;
        highRight = false;
        midLeft = false;
        midMid = false;
        midRight = true;
        hybridLeft = false;
        hybridMid = false;
        hybridRight = false;
    }

    public void setMidMid(){
        pX = Constants.VisionConstants.MID_MID_X;
        pY = Constants.VisionConstants.MID_MID_Y;
        pR = Constants.VisionConstants.MID_MID_DISTANCE;
        highLeft = false;
        highMid = false;
        highRight = false;
        midLeft = false;
        midMid = true;
        midRight = false;
        hybridLeft = false;
        hybridMid = false;
        hybridRight = false;
    }

    public void setHybridMid(){
        pX = Constants.VisionConstants.MID_HYBRID_X;
        pY = Constants.VisionConstants.MID_HYBRID_Y;
        pR = Constants.VisionConstants.MID_HYBRID_DISTANCE;
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
        pX = Constants.VisionConstants.LEFT_HYBRID_X;
        pY = Constants.VisionConstants.LEFT_HYBRID_Y;
        pR = Constants.VisionConstants.HYBRID_DISTANCE;
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
        pX = Constants.VisionConstants.RIGHT_HYBRID_X;
        pY = Constants.VisionConstants.RIGHT_HYBRID_Y;
        pR = Constants.VisionConstants.HYBRID_DISTANCE;
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

    public boolean closeEnough(){
        if(dP < Constants.VisionConstants.MAX_EXTEND_LENGTH + 0.54){
            return true;
        }
        else{
            return false;
        }
    }

    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }

    public double getAngle(){
        return angle;
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

    @Override
    public void periodic(){
        d = getArmStuff().getFirst();
        a = getArmStuff().getSecond();
        distanceBoard.setDouble(d);
        angleBoard.setDouble(a);
        if(!photonCamera.isConnected()){
            s_LED.bad();
        }
        else if(photonCamera.isConnected()){
            s_LED.notBad();
        }
        if(closeEnough()){
            s_LED.green();
        }
        else if(photonCamera.getLatestResult().getBestTarget() != null){
            s_LED.white();
        }
        else if(photonCamera.getLatestResult().getBestTarget() == null){
            s_LED.red();
        }
    }
}