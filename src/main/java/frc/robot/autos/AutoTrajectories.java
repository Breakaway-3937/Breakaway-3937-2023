package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants;

public class AutoTrajectories {

    private Trajectory leaveCommunity0Blue, leaveCommunity1Blue, leaveCommunity2Blue,
                             scoreLeave0Blue, scoreLeave1Blue, scoreLeave2Blue,
                             scoreTwice0Blue, scoreTwice1Blue, scoreTwice2Blue,
                             scoreThree0Blue, scoreThree1Blue, scoreThree2Blue,
                             leaveCharge0Blue, leaveCharge1Blue, leaveCharge2Blue,
                             scoreCharge0Blue, scoreCharge1Blue, scoreCharge2Blue,
                             scoreTwiceCharge0Blue, scoreTwiceCharge1Blue, scoreTwiceCharge2Blue,
                             scoreThreeCharge0Blue, scoreThreeCharge1Blue, scoreThreeCharge2Blue;

    private Trajectory leaveCommunity0Red, leaveCommunity1Red, leaveCommunity2Red,
                             scoreLeave0Red, scoreLeave1Red, scoreLeave2Red,
                             scoreTwice0Red, scoreTwice1Red, scoreTwice2Red,
                             scoreThree0Red, scoreThree1Red, scoreThree2Red,
                             leaveCharge0Red, leaveCharge1Red, leaveCharge2Red,
                             scoreCharge0Red, scoreCharge1Red, scoreCharge2Red,
                             scoreTwiceCharge0Red, scoreTwiceCharge1Red, scoreTwiceCharge2Red,
                             scoreThreeCharge0Red, scoreThreeCharge1Red, scoreThreeCharge2Red;

    private final TrajectoryConfig config;

    public AutoTrajectories() {
        config = new TrajectoryConfig(Constants.Auto.KMAX_SPEED_METERS_PER_SECOND, Constants.Auto.KMAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
                .setKinematics(Constants.DriveTrain.SWERVE_KINEMATICS);
    }

    public Trajectory getLeaveCommunity0Blue() {
        leaveCommunity0Blue = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(2.34, 0.74, Rotation2d.fromDegrees(0.0)),
                new Pose2d(5.95, 0.71, Rotation2d.fromDegrees(0))), 
                config);
        return leaveCommunity0Blue;
    }

    public Trajectory getLeaveCommunity1Blue() {
        leaveCommunity1Blue = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(2.12, 2.89, Rotation2d.fromDegrees(0.0)),
                new Pose2d(3.93, 2.88, Rotation2d.fromDegrees(0.0)),
                new Pose2d(5.87, 2.89, Rotation2d.fromDegrees(0.0))), 
                config);
        return leaveCommunity1Blue;
    }

    public Trajectory getLeaveCommunity2Blue() {
        leaveCommunity2Blue = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(2.21, 4.62, Rotation2d.fromDegrees(0.0)),
                new Pose2d(5.70, 4.60, Rotation2d.fromDegrees(0.0))), 
                config);
        return leaveCommunity2Blue;
    }

    public Trajectory getScoreLeave0Blue() {
        scoreLeave0Blue = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(2.12, 1.05, Rotation2d.fromDegrees(180.0)),
                new Pose2d(6.11, 0.72, Rotation2d.fromDegrees(180.0))), 
                config);
        return scoreLeave0Blue;
    }

    public Trajectory getScoreLeave1Blue(){
        scoreLeave1Blue = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(2.10, 2.76, Rotation2d.fromDegrees(180.0)),
                new Pose2d(3.89, 2.68, Rotation2d.fromDegrees(180.0)),
                new Pose2d(5.70, 2.64, Rotation2d.fromDegrees(180.0))), 
                config);
        return scoreLeave1Blue;
    }

    public Trajectory getScoreLeave2Blue(){
        scoreLeave2Blue = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(2.16, 4.43, Rotation2d.fromDegrees(180.0)),
                new Pose2d(4.52, 4.64, Rotation2d.fromDegrees(180.0))), 
                config);
        return scoreLeave2Blue;
    }

    public Trajectory getScoreTwice0Blue(){
        scoreTwice0Blue = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(2.03, 1.08, Rotation2d.fromDegrees(180.0)),
                new Pose2d(6.25, 0.91, Rotation2d.fromDegrees(0.0)),
                new Pose2d(2.03, 1.08, Rotation2d.fromDegrees(180.0))), 
                config);
        return scoreTwice0Blue;
    }

    public Trajectory getScoreTwice1Blue(){
        scoreTwice1Blue = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(2.13, 2.76, Rotation2d.fromDegrees(180.0)),
                new Pose2d(6.33, 3.32, Rotation2d.fromDegrees(0.0)),
                new Pose2d(2.13, 2.76, Rotation2d.fromDegrees(180.0))), 
                config);
        return scoreTwice1Blue;
    }

    public Trajectory getScoreTwice2Blue(){
        scoreTwice2Blue = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(2.13, 4.42, Rotation2d.fromDegrees(180.0)),
                new Pose2d(6.23, 4.55, Rotation2d.fromDegrees(0.0)),
                new Pose2d(2.13, 4.42, Rotation2d.fromDegrees(180.0))), 
                config);
        return scoreTwice2Blue;
    }

    public Trajectory getScoreThree0Blue(){
        scoreThree0Blue = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(2.0, 1.02, Rotation2d.fromDegrees(180.0)),
                new Pose2d(4.68, 0.91, Rotation2d.fromDegrees(0.0)),
                new Pose2d(6.37, 0.91, Rotation2d.fromDegrees(0.0)),
                new Pose2d(4.68, 0.91, Rotation2d.fromDegrees(180.0)),
                new Pose2d(2.0, 1.02, Rotation2d.fromDegrees(180.0)),
                new Pose2d(4.68, 0.91, Rotation2d.fromDegrees(0.0)),
                new Pose2d(6.38, 2.11, Rotation2d.fromDegrees(0.0)),
                new Pose2d(4.68, 0.91, Rotation2d.fromDegrees(180.0)),
                new Pose2d(2.0, 1.02, Rotation2d.fromDegrees(180.0))), 
                config);
        return scoreThree0Blue;
    }

    public Trajectory getScoreThree1Blue(){
        scoreThree1Blue = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(2.05, 2.76, Rotation2d.fromDegrees(180.0)),
                new Pose2d(3.93,2.73, Rotation2d.fromDegrees(0.0)),
                new Pose2d(6.25, 2.12, Rotation2d.fromDegrees(0.0)),
                new Pose2d(3.93, 2.73, Rotation2d.fromDegrees(180.0)),
                new Pose2d(2.05, 2.76, Rotation2d.fromDegrees(180.0)),
                new Pose2d(3.93, 2.73, Rotation2d.fromDegrees(0.0)),
                new Pose2d(6.29, 3.34, Rotation2d.fromDegrees(0.0)),
                new Pose2d(3.93, 2.73, Rotation2d.fromDegrees(180.0)),
                new Pose2d(2.05, 2.76, Rotation2d.fromDegrees(180.0))), 
                config);
        return scoreThree1Blue;
    }

    public Trajectory getScoreThree2Blue(){
        scoreThree2Blue = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(2.11, 4.43, Rotation2d.fromDegrees(180.0)),
                new Pose2d(4.15, 4.65, Rotation2d.fromDegrees(0.0)),
                new Pose2d(6.35,4.63, Rotation2d.fromDegrees(0.0)),
                new Pose2d(4.15, 4.65, Rotation2d.fromDegrees(180.0)),
                new Pose2d(2.11, 4.43, Rotation2d.fromDegrees(180.0)),
                new Pose2d(4.15, 4.65, Rotation2d.fromDegrees(0)),
                new Pose2d(6.38, 3.44, Rotation2d.fromDegrees(0.0)),
                new Pose2d(4.15, 4.65, Rotation2d.fromDegrees(180))), 
                config);
        return scoreThree2Blue;
    }

    public Trajectory getLeaveCharge0Blue(){
        leaveCharge0Blue = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(2.18, 0.75, Rotation2d.fromDegrees(0.0)),
                new Pose2d(7.1, 0.93, Rotation2d.fromDegrees(0.0)),
                new Pose2d(3.89, 2.41, Rotation2d.fromDegrees(0.0))), 
                config);
        return leaveCharge0Blue;
    }

    public Trajectory getLeaveCharge1Blue(){
        leaveCharge1Blue = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(2.02, 2.77, Rotation2d.fromDegrees(0.0)),
                        new Pose2d(3.89, 2.81, Rotation2d.fromDegrees(0.0)),
                        new Pose2d(5.81, 2.76, Rotation2d.fromDegrees(0.0)),
                        new Pose2d(3.89, 2.81, Rotation2d.fromDegrees(0.0))), 
                        config);
        return leaveCharge1Blue;
    }

    public Trajectory getLeaveCharge2Blue(){
        leaveCharge2Blue = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(2.08, 4.38, Rotation2d.fromDegrees(0.0)),
                new Pose2d(6.35, 4.61, Rotation2d.fromDegrees(0.0)),
                new Pose2d(3.93, 2.81, Rotation2d.fromDegrees(0.0))), 
                config);
        return leaveCharge2Blue;
    }

    public Trajectory getScoreCharge0Blue(){
        scoreCharge0Blue = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(1.93, 1.04, Rotation2d.fromDegrees(180)),
                new Pose2d(6.36, 1.29, Rotation2d.fromDegrees(180.0)),
                new Pose2d(3.93, 2.33, Rotation2d.fromDegrees(180.0))), 
                config);
        return scoreCharge0Blue;
    }

    public Trajectory getScoreCharge1Blue(){
        scoreCharge1Blue = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(1.93, 2.74, Rotation2d.fromDegrees(180)),
                new Pose2d(3.92, 2.75, Rotation2d.fromDegrees(180.0))), 
                config);
        return scoreCharge1Blue;
    }

    public Trajectory getScoreCharge2Blue(){
        scoreCharge2Blue = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(1.98, 4.37, Rotation2d.fromDegrees(180)),
                new Pose2d(6.49, 4.06, Rotation2d.fromDegrees(180.0)),
                new Pose2d(3.90, 3.45, Rotation2d.fromDegrees(180.0))), 
                config);
        return scoreCharge2Blue;
    }

    public Trajectory getScoreTwiceCharge0Blue(){
        scoreTwiceCharge0Blue = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(1.96, 1.06, Rotation2d.fromDegrees(180.0)),
                new Pose2d(7.06, 0.92, Rotation2d.fromDegrees(0.0)),
                new Pose2d(1.96, 1.06, Rotation2d.fromDegrees(180.0)),
                new Pose2d(7.06, 0.92, Rotation2d.fromDegrees(180.0)),
                new Pose2d(3.87, 2.19, Rotation2d.fromDegrees(180.0))), 
                config);
        return scoreTwiceCharge0Blue;
    }

    public Trajectory getScoreTwiceCharge1Blue(){
        scoreTwiceCharge1Blue = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(1.91, 2.75, Rotation2d.fromDegrees(180.0)),
                new Pose2d(3.89, 2.79, Rotation2d.fromDegrees(0.0)),
                new Pose2d(6.64, 2.23, Rotation2d.fromDegrees(0.0)),
                new Pose2d(3.89, 2.79, Rotation2d.fromDegrees(180.0)),
                new Pose2d(1.91, 2.72, Rotation2d.fromDegrees(180.0)),
                new Pose2d(3.89, 2.79, Rotation2d.fromDegrees(180.0))), 
                config);
        return scoreTwiceCharge1Blue;
    }

    public Trajectory getScoreTwiceCharge2Blue(){
        scoreTwiceCharge2Blue = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(1.93, 4.40, Rotation2d.fromDegrees(180.0)),
                new Pose2d(6.50, 4.62, Rotation2d.fromDegrees(0.0)),
                new Pose2d(1.93, 4.40,  Rotation2d.fromDegrees(180.0)),
                new Pose2d(6.50, 4.62, Rotation2d.fromDegrees(180.0)),
                new Pose2d(3.90, 3.31, Rotation2d.fromDegrees(180.0))), 
                config);
        return scoreTwiceCharge2Blue;
    }

    public Trajectory getScoreThreeCharge0Blue(){
        scoreThreeCharge0Blue = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(2.05, 1.03, Rotation2d.fromDegrees(180.0)),
                new Pose2d(6.58, 0.92, Rotation2d.fromDegrees(0)),
                new Pose2d(2.05, 1.03, Rotation2d.fromDegrees(180)),
                new Pose2d(6.64, 1.64, Rotation2d.fromDegrees(51.84)),
                new Pose2d(2.05, 1.03, Rotation2d.fromDegrees(180)),
                new Pose2d(3.88, 2.33, Rotation2d.fromDegrees(180.0))), 
                config);
        return scoreThreeCharge0Blue;
    }

    public Trajectory getScoreThreeCharge1Blue(){
        scoreThreeCharge1Blue = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(1.95, 3.03, Rotation2d.fromDegrees(180.0)),
                new Pose2d(3.93, 2.86, Rotation2d.fromDegrees(0.0)),
                new Pose2d(6.62, 2.08, Rotation2d.fromDegrees(180.0)),
                new Pose2d(3.93, 2.86, Rotation2d.fromDegrees(180.0)),
                new Pose2d(1.95, 3.03, Rotation2d.fromDegrees(180.0)),
                new Pose2d(3.93, 2.86, Rotation2d.fromDegrees(0.0)),
                new Pose2d(6.45, 3.34, Rotation2d.fromDegrees(0.0)),
                new Pose2d(3.93, 2.86, Rotation2d.fromDegrees(180.0)),
                new Pose2d(1.95, 3.03, Rotation2d.fromDegrees(180.0)),
                new Pose2d(3.93, 2.86, Rotation2d.fromDegrees(180.0))), 
                config);
        return scoreThreeCharge1Blue;
    }

    public Trajectory getScoreThreeCharge2Blue(){
        scoreThreeCharge2Blue = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(1.95, 4.39, Rotation2d.fromDegrees(180.0)),
                new Pose2d(6.40, 4.63, Rotation2d.fromDegrees(0.0)),
                new Pose2d(1.95, 4.39, Rotation2d.fromDegrees(180.0)),
                new Pose2d(6.42, 3.82, Rotation2d.fromDegrees(-35.07)),
                new Pose2d(1.95, 4.39, Rotation2d.fromDegrees(180.0)),
                new Pose2d(6.40, 4.63, Rotation2d.fromDegrees(180.0)),
                new Pose2d(3.91, 3.44, Rotation2d.fromDegrees(180.0))), 
                config);
        return scoreThreeCharge2Blue;
    }

    public Trajectory getLeaveCommunity0Red() {
        leaveCommunity0Red = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(16.5 - 2.34, 0.74, Rotation2d.fromDegrees(0.0)),
                new Pose2d(16.5 - 5.95, 0.71, Rotation2d.fromDegrees(0))), 
                config);
        return leaveCommunity0Red;
    }

    public Trajectory getLeaveCommunity1Red() {
        leaveCommunity1Red = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(16.5 - 2.12, 2.89, Rotation2d.fromDegrees(0.0)),
                new Pose2d(16.5 - 3.93, 2.88, Rotation2d.fromDegrees(0.0)),
                new Pose2d(16.5 - 5.87, 2.89, Rotation2d.fromDegrees(0.0))), 
                config);
        return leaveCommunity1Red;
    }

    public Trajectory getLeaveCommunity2Red() {
        leaveCommunity2Red = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(16.5 - 2.21, 4.62, Rotation2d.fromDegrees(0.0)),
                new Pose2d(16.5 - 5.70, 4.60, Rotation2d.fromDegrees(0.0))), 
                config);
        return leaveCommunity2Red;
    }

    public Trajectory getScoreLeave0Red() {
        scoreLeave0Red = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(16.5 - 2.12, 1.05, Rotation2d.fromDegrees(180.0)),
                new Pose2d(16.5 - 6.11, 0.72, Rotation2d.fromDegrees(180.0))), 
                config);
        return scoreLeave0Red;
    }

    public Trajectory getScoreLeave1Red(){
        scoreLeave1Red = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(16.5 - 2.10, 2.76, Rotation2d.fromDegrees(180.0)),
                new Pose2d(16.5 - 3.89, 2.68, Rotation2d.fromDegrees(180.0)),
                new Pose2d(16.5 - 5.70, 2.64, Rotation2d.fromDegrees(180.0))), 
                config);
        return scoreLeave1Red;
    }

    public Trajectory getScoreLeave2Red(){
        scoreLeave2Red = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(16.5 - 2.16, 4.43, Rotation2d.fromDegrees(180.0)),
                new Pose2d(16.5 - 4.52, 4.64, Rotation2d.fromDegrees(180.0))), 
                config);
        return scoreLeave2Red;
    }

    public Trajectory getScoreTwice0Red(){
        scoreTwice0Red = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(16.5 - 2.03, 1.08, Rotation2d.fromDegrees(180.0)),
                new Pose2d(16.5 - 6.25, 0.91, Rotation2d.fromDegrees(0.0)),
                new Pose2d(16.5 - 2.03, 1.08, Rotation2d.fromDegrees(180.0))), 
                config);
        return scoreTwice0Red;
    }

    public Trajectory getScoreTwice1Red(){
        scoreTwice1Red = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(16.5 - 2.13, 2.76, Rotation2d.fromDegrees(180.0)),
                new Pose2d(16.5 - 6.33, 3.32, Rotation2d.fromDegrees(0.0)),
                new Pose2d(16.5 - 2.13, 2.76, Rotation2d.fromDegrees(180.0))), 
                config);
        return scoreTwice1Red;
    }

    public Trajectory getScoreTwice2Red(){
        scoreTwice2Red = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(16.5 - 2.13, 4.42, Rotation2d.fromDegrees(180.0)),
                new Pose2d(16.5 - 6.23, 4.55, Rotation2d.fromDegrees(0.0)),
                new Pose2d(16.5 - 2.13, 4.42, Rotation2d.fromDegrees(180.0))), 
                config);
        return scoreTwice2Red;
    }

    public Trajectory getScoreThree0Red(){
        scoreThree0Red = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(16.5 - 2.0, 1.02, Rotation2d.fromDegrees(180.0)),
                new Pose2d(16.5 - 4.68, 0.91, Rotation2d.fromDegrees(0.0)),
                new Pose2d(16.5 - 6.37, 0.91, Rotation2d.fromDegrees(0.0)),
                new Pose2d(16.5 - 4.68, 0.91, Rotation2d.fromDegrees(180.0)),
                new Pose2d(16.5 - 2.0, 1.02, Rotation2d.fromDegrees(180.0)),
                new Pose2d(16.5 - 4.68, 0.91, Rotation2d.fromDegrees(0.0)),
                new Pose2d(16.5 - 6.38, 2.11, Rotation2d.fromDegrees(0.0)),
                new Pose2d(16.5 - 4.68, 0.91, Rotation2d.fromDegrees(180.0)),
                new Pose2d(16.5 - 2.0, 1.02, Rotation2d.fromDegrees(180.0))), 
                config);
        return scoreThree0Red;
    }

    public Trajectory getScoreThree1Red(){
        scoreThree1Red = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(16.5 - 2.05, 2.76, Rotation2d.fromDegrees(180.0)),
                new Pose2d(16.5 - 3.93,2.73, Rotation2d.fromDegrees(0.0)),
                new Pose2d(16.5 - 6.25, 2.12, Rotation2d.fromDegrees(0.0)),
                new Pose2d(16.5 - 3.93, 2.73, Rotation2d.fromDegrees(180.0)),
                new Pose2d(16.5 - 2.05, 2.76, Rotation2d.fromDegrees(180.0)),
                new Pose2d(16.5 - 3.93, 2.73, Rotation2d.fromDegrees(0.0)),
                new Pose2d(16.5 - 6.29, 3.34, Rotation2d.fromDegrees(0.0)),
                new Pose2d(16.5 - 3.93, 2.73, Rotation2d.fromDegrees(180.0)),
                new Pose2d(16.5 - 2.05, 2.76, Rotation2d.fromDegrees(180.0))), 
                config);
        return scoreThree1Red;
    }

    public Trajectory getScoreThree2Red(){
        scoreThree2Red = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(16.5 - 2.11, 4.43, Rotation2d.fromDegrees(180.0)),
                new Pose2d(16.5 - 4.15, 4.65, Rotation2d.fromDegrees(0.0)),
                new Pose2d(16.5 - 6.35,4.63, Rotation2d.fromDegrees(0.0)),
                new Pose2d(16.5 - 4.15, 4.65, Rotation2d.fromDegrees(180.0)),
                new Pose2d(16.5 - 2.11, 4.43, Rotation2d.fromDegrees(180.0)),
                new Pose2d(16.5 - 4.15, 4.65, Rotation2d.fromDegrees(0)),
                new Pose2d(16.5 - 6.38, 3.44, Rotation2d.fromDegrees(0.0)),
                new Pose2d(16.5 - 4.15, 4.65, Rotation2d.fromDegrees(180))), 
                config);
        return scoreThree2Red;
    }

    public Trajectory getLeaveCharge0Red(){
        leaveCharge0Red = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(16.5 - 2.18, 0.75, Rotation2d.fromDegrees(0.0)),
                new Pose2d(16.5 - 7.1, 0.93, Rotation2d.fromDegrees(0.0)),
                new Pose2d(16.5 - 3.89, 2.41, Rotation2d.fromDegrees(0.0))), 
                config);
        return leaveCharge0Red;
    }

    public Trajectory getLeaveCharge1Red(){
        leaveCharge1Red = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(16.5 - 2.02, 2.77, Rotation2d.fromDegrees(0.0)),
                        new Pose2d(16.5 - 3.89, 2.81, Rotation2d.fromDegrees(0.0)),
                        new Pose2d(16.5 - 5.81, 2.76, Rotation2d.fromDegrees(0.0)),
                        new Pose2d(16.5 - 3.89, 2.81, Rotation2d.fromDegrees(0.0))), 
                        config);
        return leaveCharge1Red;
    }

    public Trajectory getLeaveCharge2Red(){
        leaveCharge2Red = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(16.5 - 2.08, 4.38, Rotation2d.fromDegrees(0.0)),
                new Pose2d(16.5 - 6.35, 4.61, Rotation2d.fromDegrees(0.0)),
                new Pose2d(16.5 - 3.93, 2.81, Rotation2d.fromDegrees(0.0))), 
                config);
        return leaveCharge2Red;
    }

    public Trajectory getScoreCharge0Red(){
        scoreCharge0Red = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(16.5 - 1.93, 1.04, Rotation2d.fromDegrees(180)),
                new Pose2d(16.5 - 6.36, 1.29, Rotation2d.fromDegrees(180.0)),
                new Pose2d(16.5 - 3.93, 2.33, Rotation2d.fromDegrees(180.0))), 
                config);
        return scoreCharge0Red;
    }

    public Trajectory getScoreCharge1Red(){
        scoreCharge1Red = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(16.5 - 1.93, 2.74, Rotation2d.fromDegrees(180)),
                new Pose2d(16.5 - 3.92, 2.75, Rotation2d.fromDegrees(180.0))), 
                config);
        return scoreCharge1Red;
    }

    public Trajectory getScoreCharge2Red(){
        scoreCharge2Red = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(16.5 - 1.98, 4.37, Rotation2d.fromDegrees(180)),
                new Pose2d(16.5 - 6.49, 4.06, Rotation2d.fromDegrees(180.0)),
                new Pose2d(16.5 - 3.90, 3.45, Rotation2d.fromDegrees(180.0))), 
                config);
        return scoreCharge2Red;
    }

    public Trajectory getScoreTwiceCharge0Red(){
        scoreTwiceCharge0Red = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(16.5 - 1.96, 1.06, Rotation2d.fromDegrees(180.0)),
                new Pose2d(16.5 - 7.06, 0.92, Rotation2d.fromDegrees(0.0)),
                new Pose2d(16.5 - 1.96, 1.06, Rotation2d.fromDegrees(180.0)),
                new Pose2d(16.5 - 7.06, 0.92, Rotation2d.fromDegrees(180.0)),
                new Pose2d(16.5 - 3.87, 2.19, Rotation2d.fromDegrees(180.0))), 
                config);
        return scoreTwiceCharge0Red;
    }

    public Trajectory getScoreTwiceCharge1Red(){
        scoreTwiceCharge1Red = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(16.5 - 1.91, 2.75, Rotation2d.fromDegrees(180.0)),
                new Pose2d(16.5 - 3.89, 2.79, Rotation2d.fromDegrees(0.0)),
                new Pose2d(16.5 - 6.64, 2.23, Rotation2d.fromDegrees(0.0)),
                new Pose2d(16.5 - 3.89, 2.79, Rotation2d.fromDegrees(180.0)),
                new Pose2d(16.5 - 1.91, 2.72, Rotation2d.fromDegrees(180.0)),
                new Pose2d(16.5 - 3.89, 2.79, Rotation2d.fromDegrees(180.0))), 
                config);
        return scoreTwiceCharge1Red;
    }

    public Trajectory getScoreTwiceCharge2Red(){
        scoreTwiceCharge2Red = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(16.5 - 1.93, 4.40, Rotation2d.fromDegrees(180.0)),
                new Pose2d(16.5 - 6.50, 4.62, Rotation2d.fromDegrees(0.0)),
                new Pose2d(16.5 - 1.93, 4.40,  Rotation2d.fromDegrees(180.0)),
                new Pose2d(16.5 - 6.50, 4.62, Rotation2d.fromDegrees(180.0)),
                new Pose2d(16.5 - 3.90, 3.31, Rotation2d.fromDegrees(180.0))), 
                config);
        return scoreTwiceCharge2Red;
    }

    public Trajectory getScoreThreeCharge0Red(){
        scoreThreeCharge0Red = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(16.5 - 2.05, 1.03, Rotation2d.fromDegrees(180.0)),
                new Pose2d(16.5 - 6.58, 0.92, Rotation2d.fromDegrees(0)),
                new Pose2d(16.5 - 2.05, 1.03, Rotation2d.fromDegrees(180)),
                new Pose2d(16.5 - 6.64, 1.64, Rotation2d.fromDegrees(51.84)),
                new Pose2d(16.5 - 2.05, 1.03, Rotation2d.fromDegrees(180)),
                new Pose2d(16.5 - 3.88, 2.33, Rotation2d.fromDegrees(180.0))), 
                config);
        return scoreThreeCharge0Red;
    }

    public Trajectory getScoreThreeCharge1Red(){
        scoreThreeCharge1Red = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(16.5 - 1.95, 3.03, Rotation2d.fromDegrees(180.0)),
                new Pose2d(16.5 - 3.93, 2.86, Rotation2d.fromDegrees(0.0)),
                new Pose2d(16.5 - 6.62, 2.08, Rotation2d.fromDegrees(180.0)),
                new Pose2d(16.5 - 3.93, 2.86, Rotation2d.fromDegrees(180.0)),
                new Pose2d(16.5 - 1.95, 3.03, Rotation2d.fromDegrees(180.0)),
                new Pose2d(16.5 - 3.93, 2.86, Rotation2d.fromDegrees(0.0)),
                new Pose2d(16.5 - 6.45, 3.34, Rotation2d.fromDegrees(0.0)),
                new Pose2d(16.5 - 3.93, 2.86, Rotation2d.fromDegrees(180.0)),
                new Pose2d(1.95, 3.03, Rotation2d.fromDegrees(180.0)),
                new Pose2d(16.5 - 3.93, 2.86, Rotation2d.fromDegrees(180.0))), 
                config);
        return scoreThreeCharge1Red;
    }

    public Trajectory getScoreThreeCharge2Red(){
        scoreThreeCharge2Red = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(16.5 - 1.95, 4.39, Rotation2d.fromDegrees(180.0)),
                new Pose2d(16.5 - 6.40, 4.63, Rotation2d.fromDegrees(0.0)),
                new Pose2d(16.5 - 1.95, 4.39, Rotation2d.fromDegrees(180.0)),
                new Pose2d(16.5 - 6.42, 3.82, Rotation2d.fromDegrees(-35.07)),
                new Pose2d(16.5 - 1.95, 4.39, Rotation2d.fromDegrees(180.0)),
                new Pose2d(16.5 - 6.40, 4.63, Rotation2d.fromDegrees(180.0)),
                new Pose2d(16.5 - 3.91, 3.44, Rotation2d.fromDegrees(180.0))), 
                config);
        return scoreThreeCharge2Red;
    }

}