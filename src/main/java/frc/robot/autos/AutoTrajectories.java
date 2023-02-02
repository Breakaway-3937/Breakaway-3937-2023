package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.util.control.SimplePathBuilder;
import frc.lib.util.control.Trajectory;
import frc.lib.util.control.TrajectoryConstraint;
import frc.robot.Constants;

public class AutoTrajectories {

    private final Trajectory leaveCommunity0Blue, leaveCommunity1Blue, leaveCommunity2Blue,
                             scoreLeave0Blue, scoreLeave1Blue, scoreLeave2Blue,
                             scoreTwice0Blue, scoreTwice1Blue, scoreTwice2Blue,
                             scoreThree0Blue, scoreThree1Blue, scoreThree2Blue,
                             leaveCharge0Blue, leaveCharge1Blue, leaveCharge2Blue,
                             scoreCharge0Blue, scoreCharge1Blue, scoreCharge2Blue,
                             scoreTwiceCharge0Blue, scoreTwiceCharge1Blue, scoreTwiceCharge2Blue,
                             scoreThreeCharge0Blue, scoreThreeCharge1Blue, scoreThreeCharge2Blue;

    public AutoTrajectories(TrajectoryConstraint[] trajectoryConstraints) {
        leaveCommunity0Blue = new Trajectory(
                new SimplePathBuilder(new Translation2d(2.34, 0.74), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(5.95, 0.71)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE);

        leaveCommunity1Blue = new Trajectory(
                new SimplePathBuilder(new Translation2d(2.12, 2.89), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(3.93, 2.88)).lineTo(new Translation2d(5.87, 2.89)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE);

        leaveCommunity2Blue = new Trajectory(
                new SimplePathBuilder(new Translation2d(2.21, 4.62), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(5.70, 4.60)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE);

        scoreLeave0Blue = new Trajectory(
                new SimplePathBuilder(new Translation2d(2.12, 1.05), Rotation2d.fromDegrees(180.0))
                        .lineTo(new Translation2d(6.11, 0.72)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE);

        scoreLeave1Blue = new Trajectory(
                new SimplePathBuilder(new Translation2d(2.10, 2.76), Rotation2d.fromDegrees(180.0))
                        .lineTo(new Translation2d(3.89, 2.68)).lineTo(new Translation2d(5.70, 2.64)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE);

        scoreLeave2Blue = new Trajectory(
                new SimplePathBuilder(new Translation2d(2.16, 4.43), Rotation2d.fromDegrees(180.0))
                        .lineTo(new Translation2d(4.52, 4.64)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE); 
    
        scoreTwice0Blue = new Trajectory(
                new SimplePathBuilder(new Translation2d(2.03, 1.08), Rotation2d.fromDegrees(180.0))
                        .lineTo(new Translation2d(6.25, 0.91), new Rotation2d(0.0)).lineTo(new Translation2d(2.03, 1.08), new Rotation2d(180.0)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE); 
        
        scoreTwice1Blue = new Trajectory(
                new SimplePathBuilder(new Translation2d(2.13, 2.76), Rotation2d.fromDegrees(180.0))
                        .lineTo(new Translation2d(6.33, 3.32), new Rotation2d(0.0)).lineTo(new Translation2d(2.13, 2.76), new Rotation2d(180.0)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE); 
        
        scoreTwice2Blue = new Trajectory(
                new SimplePathBuilder(new Translation2d(2.13, 4.42), Rotation2d.fromDegrees(180.0))
                        .lineTo(new Translation2d(6.23, 4.55), new Rotation2d(0.0)).lineTo(new Translation2d(2.13, 4.42), new Rotation2d(180.0)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE); 
        
        scoreThree0Blue = new Trajectory(
                new SimplePathBuilder(new Translation2d(2.0, 1.02), Rotation2d.fromDegrees(180.0))
                        .lineTo(new Translation2d(4.68, 0.91), new Rotation2d(0.0)).lineTo(new Translation2d(6.37, 0.91))
                        .lineTo(new Translation2d(4.68, 0.91), new Rotation2d(180.0)).lineTo(new Translation2d(2.0, 1.02))
                        .lineTo(new Translation2d(4.68, 0.91), new Rotation2d(0.0)).lineTo(new Translation2d(6.38, 2.11))
                        .lineTo(new Translation2d(4.68, 0.91), new Rotation2d(180.0)).lineTo(new Translation2d(2.0, 1.02)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE); 
        
        scoreThree1Blue = new Trajectory(
                new SimplePathBuilder(new Translation2d(2.05, 2.76), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(3.93,2.73)).lineTo(new Translation2d(6.25, 2.12))
                        .lineTo(new Translation2d(3.93, 2.73)).lineTo(new Translation2d(2.05, 2.76))
                        .lineTo(new Translation2d(3.93, 2.73)).lineTo(new Translation2d(6.29, 3.34))
                        .lineTo(new Translation2d(3.93, 2.73)).lineTo(new Translation2d(2.05, 2.76)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE); 
        
        scoreThree2Blue = new Trajectory(
                new SimplePathBuilder(new Translation2d(2.11, 4.43), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(6.29,3.42)).lineTo(new Translation2d(2.11, 4.43))
                        .lineTo(new Translation2d(6.25, 4.56)).lineTo(new Translation2d(2.11, 4.43)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE); 
        
        leaveCharge0Blue = new Trajectory(
                new SimplePathBuilder(new Translation2d(2.18, 0.75), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(6.16, 0.97)).lineTo(new Translation2d(3.89, 2.41)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE);
        
        leaveCharge1Blue = new Trajectory(
                new SimplePathBuilder(new Translation2d(2.02, 2.77), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(3.89, 2.81)).lineTo(new Translation2d(5.81, 2.76))
                        .lineTo(new Translation2d(3.89, 2.81)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE);
        
        leaveCharge2Blue = new Trajectory(
                new SimplePathBuilder(new Translation2d(2.08, 4.38), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(6.19, 4.70)).lineTo(new Translation2d(3.93, 2.81)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE);

        scoreCharge0Blue = new Trajectory(
                new SimplePathBuilder(new Translation2d(1.93, 1.04), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(5.89, 1.48)).lineTo(new Translation2d(3.93, 2.33)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE);
        
        scoreCharge1Blue = new Trajectory(
                new SimplePathBuilder(new Translation2d(1.93, 2.74), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(3.92, 2.75)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE);

        scoreCharge2Blue = new Trajectory(
                new SimplePathBuilder(new Translation2d(1.98, 4.37), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(6.04, 4.31)).lineTo(new Translation2d(3.90, 3.45)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE);

        scoreTwiceCharge0Blue = new Trajectory(
                new SimplePathBuilder(new Translation2d(1.96, 1.06), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(7.06, 0.92)).lineTo(new Translation2d(1.96, 1.06))
                        .lineTo(new Translation2d(6.18, 1.92)).lineTo(new Translation2d(3.87, 2.19)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE);
        
        scoreTwiceCharge1Blue = new Trajectory(
                new SimplePathBuilder(new Translation2d(1.91, 2.75), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(3.89, 2.79)).lineTo(new Translation2d(6.64, 2.23))
                        .lineTo(new Translation2d(3.89, 2.79)).lineTo(new Translation2d(1.91, 2.72))
                        .lineTo(new Translation2d(3.89, 2.79)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE);
        
        scoreTwiceCharge2Blue = new Trajectory(
                new SimplePathBuilder(new Translation2d(1.93, 4.40), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(6.50, 4.62)).lineTo(new Translation2d(1.93, 4.40))
                        .lineTo(new Translation2d(5.92, 4.04)).lineTo(new Translation2d(3.90, 3.31)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE);
        
        scoreThreeCharge0Blue = new Trajectory(
                new SimplePathBuilder(new Translation2d(2.05, 1.03), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(6.58, 0.92)).lineTo(new Translation2d(2.05, 1.03))
                        .lineTo(new Translation2d(6.64, 1.64)).lineTo(new Translation2d(2.05, 1.03))
                        .lineTo(new Translation2d(3.88, 2.33)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE);
        
        scoreThreeCharge1Blue = new Trajectory(
                new SimplePathBuilder(new Translation2d(1.95, 3.03), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(3.93, 2.86)).lineTo(new Translation2d(6.62, 2.08))
                        .lineTo(new Translation2d(3.93, 2.86)).lineTo(new Translation2d(1.95, 3.03))
                        .lineTo(new Translation2d(3.93, 2.86)).lineTo(new Translation2d(6.45, 3.34))
                        .lineTo(new Translation2d(3.93, 2.86)).lineTo(new Translation2d(1.95, 3.03))
                        .lineTo(new Translation2d(3.93, 2.86)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE);
        
        scoreThreeCharge2Blue = new Trajectory(
                new SimplePathBuilder(new Translation2d(1.95, 4.39), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(6.42, 3.82)).lineTo(new Translation2d(1.95, 4.39))
                        .lineTo(new Translation2d(6.40, 4.63)).lineTo(new Translation2d(1.95, 4.39))
                        .lineTo(new Translation2d(5.41, 4.39)).lineTo(new Translation2d(3.91, 3.44)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE);
        }

    public Trajectory getLeaveCommunity0Blue() {
        return leaveCommunity0Blue;
    }

    public Trajectory getLeaveCommunity1Blue() {
        return leaveCommunity1Blue;
    }

    public Trajectory getLeaveCommunity2Blue() {
        return leaveCommunity2Blue;
    }

    public Trajectory getScoreLeave0Blue() {
        return scoreLeave0Blue;
    }

    public Trajectory getScoreLeave1Blue(){
        return scoreLeave1Blue;
    }

    public Trajectory getScoreLeave2Blue(){
        return scoreLeave2Blue;
    }

    public Trajectory getScoreTwice0Blue(){
        return scoreTwice0Blue;
    }

    public Trajectory getScoreTwice1Blue(){
        return scoreTwice1Blue;
    }

    public Trajectory getScoreTwice2Blue(){
        return scoreTwice2Blue;
    }

    public Trajectory getScoreThree0Blue(){
        return scoreThree0Blue;
    }

    public Trajectory getScoreThree1Blue(){
        return scoreThree1Blue;
    }

    public Trajectory getScoreThree2Blue(){
        return scoreThree2Blue;
    }

    public Trajectory getLeaveCharge0Blue(){
        return leaveCharge0Blue;
    }

    public Trajectory getLeaveCharge1Blue(){
        return leaveCharge1Blue;
    }

    public Trajectory getLeaveCharge2Blue(){
        return leaveCharge2Blue;
    }

    public Trajectory getScoreCharge0Blue(){
        return scoreCharge0Blue;
    }

    public Trajectory getScoreCharge1Blue(){
        return scoreCharge1Blue;
    }

    public Trajectory getScoreCharge2Blue(){
        return scoreCharge2Blue;
    }

    public Trajectory getScoreTwiceCharge0Blue(){
        return scoreTwiceCharge0Blue;
    }

    public Trajectory getScoreTwiceCharge1Blue(){
        return scoreTwiceCharge1Blue;
    }

    public Trajectory getScoreTwiceCharge2Blue(){
        return scoreTwiceCharge2Blue;
    }

    public Trajectory getScoreThreeCharge0Blue(){
        return scoreThreeCharge0Blue;
    }

    public Trajectory getScoreThreeCharge1Blue(){
        return scoreThreeCharge1Blue;
    }

    public Trajectory getScoreThreeCharge2Blue(){
        return scoreThreeCharge2Blue;
    }
}