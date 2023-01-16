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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

public class PhotonCameraWrapper extends SubsystemBase{
    public PhotonCamera photonCamera;
    public RobotPoseEstimator robotPoseEstimator;
    public AprilTagFieldLayout atfl;
    private GenericEntry poseX, poseY;
    private double x, y;
    private Pair<Pose2d, Double> pair;
    private Pose2d pose2d;

    public PhotonCameraWrapper() {
        poseX = Shuffleboard.getTab("SyrupTag").add("Pose X", x).withPosition(0, 0).getEntry();
        poseY = Shuffleboard.getTab("SyrupTag").add("Pose Y", y).withPosition(0, 0).getEntry();
        try {
            atfl = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
        }

        // Forward Camera
        photonCamera =
                new PhotonCamera(
                        VisionConstants
                                .CAMERA_NAME); // Change the name of your camera here to whatever it is in the
        // PhotonVision UI.

        // ... Add other cameras here

        // Assemble the list of cameras & mount locations
        var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
        camList.add(new Pair<PhotonCamera, Transform3d>(photonCamera, VisionConstants.ROBOT_TO_CAM));

        robotPoseEstimator =
                new RobotPoseEstimator(atfl, PoseStrategy.AVERAGE_BEST_TARGETS, camList);
    }

    /**
     * @param estimatedRobotPose The current best guess at robot PoseX
     * @return A pair of the fused camera observations to a single Pose2d on the field, and the time
     *     of the observation. Assumes a planar field and the robot is always firmly on the ground
     */
    public Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        robotPoseEstimator.setReferencePose(prevEstimatedRobotPose);

        double currentTime = Timer.getFPGATimestamp();
        Optional<Pair<Pose3d, Double>> result = robotPoseEstimator.update();
        if (result.isPresent()) {
            return new Pair<Pose2d, Double>(
                    result.get().getFirst().toPose2d(), currentTime - result.get().getSecond());
        } else {
            return new Pair<Pose2d, Double>(null, 0.0);
        }
    }

    @Override
    public void periodic(){
        for(int i = 1; i < 9; i++){
            if(photonCamera.getLatestResult().getBestTarget().getFiducialId() == i){
                pose2d = new Pose2d(1, 1, new Rotation2d(1));
                pair = getEstimatedGlobalPose(pose2d);
                pose2d = pair.getFirst();
                double x = pose2d.getX();
                double y = pose2d.getY();
                poseX.setDouble(x);
                poseY.setDouble(y);
            }
        }
    }
}
