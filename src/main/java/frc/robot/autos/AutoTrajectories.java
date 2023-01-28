package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.util.control.SimplePathBuilder;
import frc.lib.util.control.Trajectory;
import frc.lib.util.control.TrajectoryConstraint;
import frc.robot.Constants;

public class AutoTrajectories {

    private final Trajectory leaveCommunity0, leaveCommunity1, leaveCommunity2,
        scoreLeave0, scoreLeaveBridge, scoreLeave1,
        scoreTwice0, scoreTwice1, scoreTwice2,
        scoreThree0, scoreThree1, scoreThree2,
        leaveCharge0, leaveCharge1, leaveCharge2,
        scoreCharge0, scoreCharge1, scoreCharge2,
        scoreTwiceCharge0, scoreTwiceCharge1, scoreTwiceCharge2,
        scoreThreeCharge0, scoreThreeCharge1, scoreThreeCharge2;

    public AutoTrajectories(TrajectoryConstraint[] trajectoryConstraints) {
        leaveCommunity0 = new Trajectory(
                new SimplePathBuilder(new Translation2d(2.34, 0.74), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(5.95, 0.71)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE);

        leaveCommunity1 = new Trajectory(
                new SimplePathBuilder(new Translation2d(2.12, 2.89), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(3.93, 2.88)).lineTo(new Translation2d(5.87, 2.89)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE);

        leaveCommunity2 = new Trajectory(
                new SimplePathBuilder(new Translation2d(2.21, 4.62), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(5.70, 4.60)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE);

        scoreLeave0 = new Trajectory(
                new SimplePathBuilder(new Translation2d(2.12, 1.05), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(6.11, 0.72)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE);

        scoreLeaveBridge = new Trajectory(
                new SimplePathBuilder(new Translation2d(2.10, 2.76), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(3.89, 2.68)).lineTo(new Translation2d(5.70, 2.64)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE);

        scoreLeave1 = new Trajectory(
                new SimplePathBuilder(new Translation2d(2.16, 4.43), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(4.52, 4.64)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE); 
    
        scoreTwice0 = new Trajectory(
                new SimplePathBuilder(new Translation2d(2.03, 1.08), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(6.25, 0.91)).lineTo(new Translation2d(2.03, 1.08)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE); 
        
        scoreTwice1 = new Trajectory(
                new SimplePathBuilder(new Translation2d(2.13, 2.76), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(6.33, 3.32)).lineTo(new Translation2d(2.13, 2.76)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE); 
        
        scoreTwice2 = new Trajectory(
                new SimplePathBuilder(new Translation2d(2.13, 4.42), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(6.23, 4.55)).lineTo(new Translation2d(2.13, 4.42)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE); 
        
        scoreThree0 = new Trajectory(
                new SimplePathBuilder(new Translation2d(2.16, 1.06), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(6.20, 2.14)).lineTo(new Translation2d(2.16, 1.06))
                        .lineTo(new Translation2d(6.18, 0.91)).lineTo(new Translation2d(2.16, 1.06)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE); 
        
        scoreThree1 = new Trajectory(
                new SimplePathBuilder(new Translation2d(2.05, 2.76), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(3.93,2.73)).lineTo(new Translation2d(6.25, 2.12))
                        .lineTo(new Translation2d(3.93, 2.73)).lineTo(new Translation2d(2.05, 2.76))
                        .lineTo(new Translation2d(3.93, 2.73)).lineTo(new Translation2d(6.29, 3.34))
                        .lineTo(new Translation2d(3.93, 2.73)).lineTo(new Translation2d(2.05, 2.76)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE); 
        
        scoreThree2 = new Trajectory(
                new SimplePathBuilder(new Translation2d(2.11, 4.43), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(6.29,3.42)).lineTo(new Translation2d(2.11, 4.43))
                        .lineTo(new Translation2d(6.25, 4.56)).lineTo(new Translation2d(2.11, 4.43)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE); 
        
        leaveCharge0 = new Trajectory(
                new SimplePathBuilder(new Translation2d(2.18, 0.75), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(6.16, 0.97)).lineTo(new Translation2d(3.89, 2.41)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE);
        
        leaveCharge1 = new Trajectory(
                new SimplePathBuilder(new Translation2d(2.02, 2.77), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(3.89, 2.81)).lineTo(new Translation2d(5.81, 2.76))
                        .lineTo(new Translation2d(3.89, 2.81)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE);
        
        leaveCharge2 = new Trajectory(
                new SimplePathBuilder(new Translation2d(2.08, 4.38), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(5.03, 4.71)).lineTo(new Translation2d(3.93, 2.81)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE);

        scoreCharge0 = new Trajectory(
                new SimplePathBuilder(new Translation2d(1.93, 1.04), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(5.89, 1.48)).lineTo(new Translation2d(3.93, 2.33)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE);
        
        scoreCharge1 = new Trajectory(
                new SimplePathBuilder(new Translation2d(1.93, 2.74), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(3.92, 2.75)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE);

        scoreCharge2 = new Trajectory(
                new SimplePathBuilder(new Translation2d(1.98, 4.37), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(6.04, 4.31)).lineTo(new Translation2d(3.90, 3.45)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE);

        scoreTwiceCharge0 = new Trajectory(
                new SimplePathBuilder(new Translation2d(1.96, 1.06), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(7.06, 0.92)).lineTo(new Translation2d(1.96, 1.06))
                        .lineTo(new Translation2d(6.18, 1.92)).lineTo(new Translation2d(3.87, 2.19)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE);
        
        scoreTwiceCharge1 = new Trajectory(
                new SimplePathBuilder(new Translation2d(1.91, 2.75), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(3.89, 2.79)).lineTo(new Translation2d(6.64, 2.23))
                        .lineTo(new Translation2d(3.89, 2.79)).lineTo(new Translation2d(1.91, 2.72))
                        .lineTo(new Translation2d(3.89, 2.79)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE);
        
        scoreTwiceCharge2 = new Trajectory(
                new SimplePathBuilder(new Translation2d(1.93, 4.40), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(6.50, 4.62)).lineTo(new Translation2d(1.93, 4.40))
                        .lineTo(new Translation2d(5.92, 4.04)).lineTo(new Translation2d(3.90, 3.31)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE);
        
        scoreThreeCharge0 = new Trajectory(
                new SimplePathBuilder(new Translation2d(2.05, 1.03), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(6.58, 0.92)).lineTo(new Translation2d(2.05, 1.03))
                        .lineTo(new Translation2d(6.64, 1.64)).lineTo(new Translation2d(2.05, 1.03))
                        .lineTo(new Translation2d(3.88, 2.33)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE);
        
        scoreThreeCharge1 = new Trajectory(
                new SimplePathBuilder(new Translation2d(1.95, 3.03), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(3.93, 2.86)).lineTo(new Translation2d(6.62, 2.08))
                        .lineTo(new Translation2d(3.93, 2.86)).lineTo(new Translation2d(1.95, 3.03))
                        .lineTo(new Translation2d(3.93, 2.86)).lineTo(new Translation2d(6.45, 3.34))
                        .lineTo(new Translation2d(3.93, 2.86)).lineTo(new Translation2d(1.95, 3.03))
                        .lineTo(new Translation2d(3.93, 2.86)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE);
        
        scoreThreeCharge2 = new Trajectory(
                new SimplePathBuilder(new Translation2d(1.95, 4.39), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(6.42, 3.82)).lineTo(new Translation2d(1.95, 4.39))
                        .lineTo(new Translation2d(6.40, 4.63)).lineTo(new Translation2d(1.95, 4.39))
                        .lineTo(new Translation2d(5.41, 4.39)).lineTo(new Translation2d(3.91, 3.44)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE);
        }

    public Trajectory getLeaveCommunity0() {
        return leaveCommunity0;
    }

    public Trajectory getLeaveCommunity1() {
        return leaveCommunity1;
    }

    public Trajectory getLeaveCommunity2() {
        return leaveCommunity2;
    }

    public Trajectory getscoreLeave() {
        return scoreLeave0;
    }

    public Trajectory getScoreLeaveBridge(){
        return scoreLeaveBridge;
    }

    public Trajectory getScoreLeave1(){
        return scoreLeave1;
    }

    public Trajectory getScoreTwice0(){
        return scoreTwice0;
    }

    public Trajectory getScoreTwice1(){
        return scoreTwice1;
    }

    public Trajectory getScoreTwice2(){
        return scoreTwice2;
    }

    public Trajectory getScoreThree0(){
        return scoreThree0;
    }

    public Trajectory getScoreThree1(){
        return scoreThree1;
    }

    public Trajectory getScoreThree2(){
        return scoreThree2;
    }

    public Trajectory getLeaveCharge0(){
        return leaveCharge0;
    }

    public Trajectory getLeaveChrage1(){
        return leaveCharge1;
    }

    public Trajectory getLeaveCharge2(){
        return leaveCharge2;
    }

    public Trajectory getScoreCharge0(){
        return scoreCharge0;
    }

    public Trajectory getScoreCharge1(){
        return scoreCharge1;
    }

    public Trajectory getScoreCharge2(){
        return scoreCharge2;
    }

    public Trajectory getScoreTwiceCharge0(){
        return scoreTwiceCharge0;
    }

    public Trajectory getScoreTwiceCharge1(){
        return scoreTwiceCharge1;
    }

    public Trajectory getScoreTwiceCharge2(){
        return scoreTwiceCharge2;
    }

    public Trajectory getScoreThreeCharge0(){
        return scoreThreeCharge0;
    }

    public Trajectory getScoreThreeCharge1(){
        return scoreThreeCharge1;
    }

    public Trajectory getScoreThreeCharge2(){
        return scoreThreeCharge2;
    }
}