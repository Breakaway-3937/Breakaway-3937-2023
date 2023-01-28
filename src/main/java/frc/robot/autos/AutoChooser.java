package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.util.control.Path;
import frc.lib.util.control.Trajectory;
import frc.robot.RobotContainer;
import frc.robot.commands.FollowTrajectoryCommand;

public class AutoChooser {
    private final AutoTrajectories trajectories;

    private final SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();

    public AutoChooser(AutoTrajectories trajectories) {
        this.trajectories = trajectories;
        autonomousModeChooser.addOption("Do Nothing", AutonomousMode.DO_NOTHING);
    }

    public SendableChooser<AutonomousMode> getModeChooser() {
        return autonomousModeChooser;
    }

    public Command getDoNothing(RobotContainer robotContainer) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, robotContainer, null);

        command.addCommands(follow(robotContainer, null));

        return command;
    }
    
    private Command follow(RobotContainer robotContainer, Trajectory trajectory) {
        return new FollowTrajectoryCommand(robotContainer.getDrivetrain(), trajectory);
    }

    public void resetRobotPose(SequentialCommandGroup command, RobotContainer robotContainer, Trajectory trajectory) {
        Path.State start = trajectory.getPath().calculate(0.0);
        command.addCommands(new InstantCommand(() -> robotContainer.getDrivetrain().setPose(new Pose2d(start.getPosition().getX(),
                start.getPosition().getY(), new Rotation2d(start.getRotation().getRadians())))));
    }

    public Command getCommand(RobotContainer robotContainer) {
        switch (autonomousModeChooser.getSelected()) {
            case DO_NOTHING :
                return getDoNothing(robotContainer);
        }
        return new InstantCommand();
    }

    private enum AutonomousMode {
        DO_NOTHING
    }
}