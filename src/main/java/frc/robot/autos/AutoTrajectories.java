package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.util.control.*;
import frc.robot.Constants;

public class AutoTrajectories {

    private final Trajectory leaveCommunity0, leaveCommunity1, leaveCommunity2;

    public AutoTrajectories(TrajectoryConstraint[] trajectoryConstraints) {
        leaveCommunity0 = new Trajectory(
                new SimplePathBuilder(new Translation2d(2.1, 0.54), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(5.8, 0.58)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE);

        leaveCommunity1 = new Trajectory(
                new SimplePathBuilder(new Translation2d(2.05, 2.65), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(3.89, 2.66)).lineTo(new Translation2d(5.85, 2.62)).build(),
                trajectoryConstraints, Constants.SAMPLE_DISTANCE);

        leaveCommunity2 = new Trajectory(
                new SimplePathBuilder(new Translation2d(2.19, 4.69), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(4.68, 4.7)).build(),
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
}