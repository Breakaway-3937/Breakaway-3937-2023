package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.util.control.*;

public class AutoTrajectories {

    private static final double SAMPLE_DISTANCE = Units.inchesToMeters(0.1);

    private final Trajectory testAutoPartOne;

    public AutoTrajectories(TrajectoryConstraint[] trajectoryConstraints) {
        testAutoPartOne = new Trajectory(
                new SimplePathBuilder(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0))
                        .lineTo(new Translation2d(Units.feetToMeters(30.0), 0.0)).build(),
                trajectoryConstraints, SAMPLE_DISTANCE);
    }

    public Trajectory getTestAutoPartOne() {
        return testAutoPartOne;
    }
}