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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

import java.io.IOException;
import java.sql.ResultSet;
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonVersion;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonCameraWrapper extends SubsystemBase{
    private PhotonCamera photonCamera;
    private PhotonPoseEstimator photonPoseEstimator;
    private AprilTagFieldLayout atfl;
    private GenericEntry poseX, poseY, distanceBoard, angleBoard, test, test1, test2, testRx, testRy;
    private double x, y, d, a;
    private Pair<Pose2d, Double> pair;
    private Pose2d pose2d = new Pose2d(0, 0, new Rotation2d(0));;
    private double rY, rX, rR, /*test doubles ->*/dP, pR, theta, testRx1, testRy1, pX, pY;

    public PhotonCameraWrapper() {
        poseX = Shuffleboard.getTab("SyrupTag").add("Pose X", x).withPosition(0, 0).getEntry();
        poseY = Shuffleboard.getTab("SyrupTag").add("Pose Y", y).withPosition(1, 0).getEntry();
        distanceBoard = Shuffleboard.getTab("SyrupTag").add("Distance", d).withPosition(2, 0).getEntry();
        angleBoard = Shuffleboard.getTab("SyrupTag").add("Angle", a).withPosition(3, 0).getEntry();
        test = Shuffleboard.getTab("SyrupTag").add("rR", a).withPosition(4, 0).getEntry();
        test1 = Shuffleboard.getTab("SyrupTag").add("dP", dP).withPosition(4, 1).getEntry();
        test2 = Shuffleboard.getTab("SyrupTag").add("theta", theta).withPosition(6, 0).getEntry();
        testRx = Shuffleboard.getTab("SyrupTag").add("testRx", testRx1).withPosition(6, 1).getEntry();
        testRy = Shuffleboard.getTab("SyrupTag").add("testRy", testRy1).withPosition(6, 2).getEntry();
        try {
            atfl = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
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
            return new Pair<Pose2d, Double>(null, 0.0);
        }
    
    }


    public Pair<Double, Double> getArmStuff(){
        var result = photonCamera.getLatestResult();
        if(result.hasTargets()){
        rY = result.getBestTarget().getBestCameraToTarget().getY();
        testRy.setDouble(rY);
        rX = result.getBestTarget().getBestCameraToTarget().getX();;
        testRx.setDouble(rX);
        rR = Math.sqrt(Math.pow(rX, 2) + Math.pow(rY, 2));
        test.setDouble(rR);
        pX = Constants.VisionConstants.HIGH_LEFT_POST_X;
        pY = Constants.VisionConstants.HIGH_LEFT_POST_Y;
        dP = Math.sqrt(Math.pow((Constants.VisionConstants.HIGH_LEFT_POST_X - rX), 2) + Math.pow((Constants.VisionConstants.HIGH_LEFT_POST_Y - rY), 2));
        test1.setDouble(dP);
        pR = Constants.VisionConstants.HIGH_DISTANCE;
        double noCos = Math.abs((Math.pow(pR, 2) - Math.pow(rR, 2) - Math.pow(dP, 2)) / (2 * (rR * dP)));
        theta = Math.toDegrees(Math.acos(noCos));
        test2.setDouble(theta);
        return new Pair<Double, Double>(dP, theta);
        }
        else{
            return new Pair<Double,Double>(0.0, 0.0);
        }
    }

    @Override
    public void periodic(){
        try{
            pair = getEstimatedGlobalPose(pose2d);
            pose2d = pair.getFirst();
            x = pose2d.getX();
            y = pose2d.getY();
            d = getArmStuff().getFirst();
            a = getArmStuff().getSecond();
            poseX.setDouble(x);
            poseY.setDouble(y);
            distanceBoard.setDouble(d);
            angleBoard.setDouble(a);
            
        }
        catch(Exception e){}
    }
}