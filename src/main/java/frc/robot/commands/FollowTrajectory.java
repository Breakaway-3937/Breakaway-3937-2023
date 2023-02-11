package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.control.Trajectory;
import frc.robot.subsystems.DriveTrain;

public class FollowTrajectory extends CommandBase {
    private final DriveTrain s_Drivetrain;

    private final Trajectory trajectory;

    public FollowTrajectory(DriveTrain s_Drivetrain, Trajectory trajectory) {
        this.s_Drivetrain = s_Drivetrain;
        this.trajectory = trajectory;

        addRequirements(s_Drivetrain);
    }

    @Override
    public void initialize() {
        s_Drivetrain.getFollower().follow(trajectory);
    }

    @Override
    public void end(boolean interrupted) {
        s_Drivetrain.getFollower().cancel();
    }

    @Override
    public boolean isFinished() {
        return s_Drivetrain.getFollower().getCurrentTrajectory().isEmpty();
    }
}