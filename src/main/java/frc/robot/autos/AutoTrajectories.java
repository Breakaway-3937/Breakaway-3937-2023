package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.Constants;

public class AutoTrajectories {

    private PathPlannerTrajectory leaveCommunity0, leaveCommunity1, leaveCommunity2,
                             scoreLeave0, scoreLeave1, scoreLeave2,
                             scoreTwice0, scoreTwice2, scoreTwice0Leave, scoreTwice2Leave,
                             leaveCharge0, leaveCharge1, leaveCharge2,
                             scoreCharge0, scoreCharge1, scoreCharge2,
                             scoreGrabCharge0, scoreGrabCharge1, scoreGrabCharge2;


    private final PathConstraints constraints, slowConstraints;

    public AutoTrajectories() {
        constraints = new PathConstraints(Constants.Auto.KMAX_SPEED_METERS_PER_SECOND, Constants.Auto.KMAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        //Coach(God(in his words)) said turn to 10
        slowConstraints = new PathConstraints(1.6, Constants.Auto.KMAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        leaveCommunity0 = PathPlanner.loadPath("LeaveCommunity0", constraints);
        leaveCommunity1 = PathPlanner.loadPath("LeaveCommunity1", slowConstraints);
        leaveCommunity2 = PathPlanner.loadPath("LeaveCommunity2", constraints);
        scoreLeave0 = PathPlanner.loadPath("ScoreLeave0", constraints);
        scoreLeave1 = PathPlanner.loadPath("ScoreLeave1", slowConstraints);
        scoreLeave2 = PathPlanner.loadPath("ScoreLeave2", constraints);
        scoreTwice0 = PathPlanner.loadPath("ScoreTwice0", constraints);
        scoreTwice0Leave = PathPlanner.loadPath("ScoreTwice0Leave", constraints);
        scoreTwice2 = PathPlanner.loadPath("ScoreTwice2", constraints);
        scoreTwice2Leave = PathPlanner.loadPath("ScoreTwice2Leave", constraints);
        leaveCharge0 = PathPlanner.loadPath("LeaveCharge0", constraints);
        leaveCharge1 = PathPlanner.loadPath("LeaveCharge1", slowConstraints);
        leaveCharge2 = PathPlanner.loadPath("LeaveCharge2", constraints);
        scoreCharge0 = PathPlanner.loadPath("ScoreCharge0", constraints);
        scoreCharge1 = PathPlanner.loadPath("ScoreCharge1", slowConstraints);
        scoreCharge2 = PathPlanner.loadPath("ScoreCharge2", constraints);
        scoreGrabCharge0 = PathPlanner.loadPath("ScoreGrabCharge0", constraints);
        scoreGrabCharge1 = PathPlanner.loadPath("ScoreGrabCharge1", slowConstraints);
        scoreGrabCharge2 = PathPlanner.loadPath("ScoreGrabCharge2", constraints);
    }

    public PathPlannerTrajectory getLeaveCommunity0() {
        return leaveCommunity0;
    }

    public PathPlannerTrajectory getLeaveCommunity1() {
        return leaveCommunity1;
    }

    public PathPlannerTrajectory getLeaveCommunity2() {
        return leaveCommunity2;
    }

    public PathPlannerTrajectory getScoreLeave0() {
        return scoreLeave0;
    }

    public PathPlannerTrajectory getScoreLeave1(){
        return scoreLeave1;
    }

    public PathPlannerTrajectory getScoreLeave2(){
        return scoreLeave2;
    }

    public PathPlannerTrajectory getScoreTwice0(){
        return scoreTwice0;
    }

    public PathPlannerTrajectory getScoreTwice2(){
        return scoreTwice2;
    }

    public PathPlannerTrajectory getScoreTwice0Leave(){
        return scoreTwice0Leave;
    }

    public PathPlannerTrajectory getScoreTwice2Leave(){
        return scoreTwice2Leave;
    }

    public PathPlannerTrajectory getLeaveCharge0(){
        return leaveCharge0;
    }

    public PathPlannerTrajectory getLeaveCharge1(){
        return leaveCharge1;
    }

    public PathPlannerTrajectory getLeaveCharge2(){
        return leaveCharge2;
    }

    public PathPlannerTrajectory getScoreCharge0(){
        return scoreCharge0;
    }

    public PathPlannerTrajectory getScoreCharge1(){
        return scoreCharge1;
    }

    public PathPlannerTrajectory getScoreCharge2(){
        return scoreCharge2;
    }

    public PathPlannerTrajectory getScoreGrabCharge0(){
        return scoreGrabCharge0;
    }

    public PathPlannerTrajectory getScoreGrabCharge1(){
        return scoreGrabCharge1;
    }

    public PathPlannerTrajectory getScoreGrabCharge2(){
        return scoreGrabCharge2;
    }
}