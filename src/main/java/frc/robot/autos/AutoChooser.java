package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.util.control.Trajectory;
import frc.robot.RobotContainer;
import frc.robot.commands.FollowTrajectoryCommand;

public class AutoChooser {
    private final AutoTrajectories trajectories;

    private final SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();

    public AutoChooser(AutoTrajectories trajectories) {
        this.trajectories = trajectories;
        autonomousModeChooser.setDefaultOption("Do Nothing", AutonomousMode.DO_NOTHING);
        autonomousModeChooser.addOption("Leave Community 0", AutonomousMode.LEAVE_COMMUNITY_0);
        autonomousModeChooser.addOption("Leave Community 1", AutonomousMode.LEAVE_COMMUNITY_1);
        autonomousModeChooser.addOption("Leave Community 2", AutonomousMode.LEAVE_COMMUNITY_2);
        autonomousModeChooser.addOption("Leave Charge 0", AutonomousMode.LEAVE_CHARGE_0);
        autonomousModeChooser.addOption("Leave Charge 1", AutonomousMode.LEAVE_CHARGE_1);
        autonomousModeChooser.addOption("Leave Charge 2", AutonomousMode.LEAVE_CHARGE_2);
        autonomousModeChooser.addOption("Score Leave 0", AutonomousMode.SCORE_LEAVE_0);
        autonomousModeChooser.addOption("Score Leave 1", AutonomousMode.SCORE_LEAVE_1);
        autonomousModeChooser.addOption("Score Leave 2", AutonomousMode.SCORE_LEAVE_2);
        autonomousModeChooser.addOption("Score Twice 0", AutonomousMode.SCORE_TWICE_0);
        autonomousModeChooser.addOption("Score Twice 1", AutonomousMode.SCORE_TWICE_1);
        autonomousModeChooser.addOption("Score Twice 2", AutonomousMode.SCORE_TWICE_2);
        autonomousModeChooser.addOption("Score Three 0", AutonomousMode.SCORE_THREE_0);
        autonomousModeChooser.addOption("Score Three 1", AutonomousMode.SCORE_THREE_1);
        autonomousModeChooser.addOption("Score Three 2", AutonomousMode.SCORE_THREE_2);
        autonomousModeChooser.addOption("Leave Charge 0", AutonomousMode.LEAVE_CHARGE_0);
        autonomousModeChooser.addOption("Leave Charge 1", AutonomousMode.LEAVE_CHARGE_1);
        autonomousModeChooser.addOption("Leave Charge 2", AutonomousMode.LEAVE_CHARGE_2);
        autonomousModeChooser.addOption("Score Charge 0", AutonomousMode.SCORE_CHARGE_0);
        autonomousModeChooser.addOption("Score Charge 1", AutonomousMode.SCORE_CHARGE_1);
        autonomousModeChooser.addOption("Score Charge 2", AutonomousMode.SCORE_CHARGE_2);
        autonomousModeChooser.addOption("Score Twice Charge 0", AutonomousMode.SCORE_TWICE_CHARGE_0);
        autonomousModeChooser.addOption("Score Twice Charge 1", AutonomousMode.SCORE_TWICE_CHARGE_1);
        autonomousModeChooser.addOption("Score Twice Charge 2", AutonomousMode.SCORE_TWICE_CHARGE_2);
        autonomousModeChooser.addOption("Score Three Charge 0", AutonomousMode.SCORE_THREE_CHARGE_0);
        autonomousModeChooser.addOption("Score Three Charge 1", AutonomousMode.SCORE_THREE_CHARGE_1);
        autonomousModeChooser.addOption("Score Three Charge 2", AutonomousMode.SCORE_THREE_CHARGE_2);
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

    public Command getLeaveCommunity0(RobotContainer robotContainer) {
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();

            resetRobotPose(command, robotContainer, trajectories.getLeaveCommunity0Blue());
    
            command.addCommands(follow(robotContainer, trajectories.getLeaveCommunity0Blue()));
    
            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, robotContainer, trajectories.getLeaveCommunity0Red());

        command.addCommands(follow(robotContainer, trajectories.getLeaveCommunity0Red()));

        return command;
        
    }

    public Command getLeaveCommunity1(RobotContainer robotContainer) {
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();

            resetRobotPose(command, robotContainer, trajectories.getLeaveCommunity1Blue());

            command.addCommands(follow(robotContainer, trajectories.getLeaveCommunity1Blue()));

            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();

            resetRobotPose(command, robotContainer, trajectories.getLeaveCommunity1Red());

            command.addCommands(follow(robotContainer, trajectories.getLeaveCommunity1Red()));

            return command;
    }

    public Command getLeaveCommunity2(RobotContainer robotContainer) {
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();

            resetRobotPose(command, robotContainer, trajectories.getLeaveCommunity1Blue());

            command.addCommands(follow(robotContainer, trajectories.getLeaveCommunity1Blue()));

            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, robotContainer, trajectories.getLeaveCommunity2Red());

        command.addCommands(follow(robotContainer, trajectories.getLeaveCommunity2Red()));

        return command;
    }

    public Command getScoreLeave0(RobotContainer robotContainer) {
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();

            resetRobotPose(command, robotContainer, trajectories.getScoreLeave0Blue());

            command.addCommands(follow(robotContainer, trajectories.getScoreLeave0Blue()));

            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, robotContainer, trajectories.getScoreLeave0Red());

        command.addCommands(follow(robotContainer, trajectories.getScoreLeave0Red()));

        return command;
    }

    public Command getScoreLeave1(RobotContainer robotContainer) {
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();

            resetRobotPose(command, robotContainer, trajectories.getScoreLeave1Blue());

            command.addCommands(follow(robotContainer, trajectories.getScoreLeave1Blue()));

            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, robotContainer, trajectories.getScoreLeave1Red());

        command.addCommands(follow(robotContainer, trajectories.getScoreLeave1Red()));

        return command;
    }

    public Command getScoreLeave2(RobotContainer robotContainer) {
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();

            resetRobotPose(command, robotContainer, trajectories.getScoreLeave2Blue());

            command.addCommands(follow(robotContainer, trajectories.getScoreLeave2Blue()));

            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, robotContainer, trajectories.getScoreLeave2Red());

        command.addCommands(follow(robotContainer, trajectories.getScoreLeave2Red()));

        return command;
    }

    public Command getScoreTwice0(RobotContainer robotContainer) {
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();

            resetRobotPose(command, robotContainer, trajectories.getScoreTwice0Blue());

            command.addCommands(follow(robotContainer, trajectories.getScoreTwice0Blue()));

            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();

            resetRobotPose(command, robotContainer, trajectories.getScoreTwice0Red());

            command.addCommands(follow(robotContainer, trajectories.getScoreTwice0Red()));

            return command;
    }

    public Command getScoreTwice1(RobotContainer robotContainer) {
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();

            resetRobotPose(command, robotContainer, trajectories.getScoreTwice1Blue());

            command.addCommands(follow(robotContainer, trajectories.getScoreTwice1Blue()));

            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, robotContainer, trajectories.getScoreTwice1Red());

        command.addCommands(follow(robotContainer, trajectories.getScoreTwice1Red()));

        return command;
    }

    public Command getScoreTwice2(RobotContainer robotContainer) {
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();

            resetRobotPose(command, robotContainer, trajectories.getScoreTwice2Blue());

            command.addCommands(follow(robotContainer, trajectories.getScoreTwice2Blue()));

            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, robotContainer, trajectories.getScoreTwice2Red());

        command.addCommands(follow(robotContainer, trajectories.getScoreTwice2Red()));

        return command;
    }
    
    public Command getScoreThree0(RobotContainer robotContainer) {
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();

            resetRobotPose(command, robotContainer, trajectories.getScoreThree0Blue());

            command.addCommands(follow(robotContainer, trajectories.getScoreThree0Blue()));

            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, robotContainer, trajectories.getScoreThree0Red());

        command.addCommands(follow(robotContainer, trajectories.getScoreThree0Red()));

        return command;
    }

    public Command getScoreThree1(RobotContainer robotContainer) {
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();

            resetRobotPose(command, robotContainer, trajectories.getScoreThree1Blue());

            command.addCommands(follow(robotContainer, trajectories.getScoreThree1Blue()));

            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, robotContainer, trajectories.getScoreThree1Red());

        command.addCommands(follow(robotContainer, trajectories.getScoreThree1Red()));

        return command;
    }

    public Command getScoreThree2(RobotContainer robotContainer) {
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();

            resetRobotPose(command, robotContainer, trajectories.getScoreThree2Blue());

            command.addCommands(follow(robotContainer, trajectories.getScoreThree2Blue()));

            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, robotContainer, trajectories.getScoreThree2Red());

        command.addCommands(follow(robotContainer, trajectories.getScoreThree2Red()));

        return command;
    }

    public Command getLeaveCharge0(RobotContainer robotContainer) {
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();

            resetRobotPose(command, robotContainer, trajectories.getLeaveCharge0Blue());

            command.addCommands(follow(robotContainer, trajectories.getLeaveCharge0Blue()));

            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, robotContainer, trajectories.getLeaveCharge0Red());

        command.addCommands(follow(robotContainer, trajectories.getLeaveCharge0Red()));

        return command;
    }

    public Command getLeaveCharge1(RobotContainer robotContainer) {
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();

            resetRobotPose(command, robotContainer, trajectories.getLeaveCharge1Blue());

            command.addCommands(follow(robotContainer, trajectories.getLeaveCharge1Blue()));

            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, robotContainer, trajectories.getLeaveCharge1Red());

        command.addCommands(follow(robotContainer, trajectories.getLeaveCharge1Red()));

        return command;
    }

    public Command getLeaveCharge2(RobotContainer robotContainer) {
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();

            resetRobotPose(command, robotContainer, trajectories.getLeaveCharge2Blue());

            command.addCommands(follow(robotContainer, trajectories.getLeaveCharge2Blue()));

            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, robotContainer, trajectories.getLeaveCharge2Red());

        command.addCommands(follow(robotContainer, trajectories.getLeaveCharge2Red()));

        return command;
    }

    public Command getScoreCharge0(RobotContainer robotContainer) {
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();

            resetRobotPose(command, robotContainer, trajectories.getScoreCharge0Blue());

            command.addCommands(follow(robotContainer, trajectories.getScoreCharge0Blue()));

            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, robotContainer, trajectories.getScoreCharge0Red());

        command.addCommands(follow(robotContainer, trajectories.getScoreCharge0Red()));

        return command;
    }

    public Command getScoreCharge1(RobotContainer robotContainer) {
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();

            resetRobotPose(command, robotContainer, trajectories.getScoreCharge1Blue());

            command.addCommands(follow(robotContainer, trajectories.getScoreCharge1Blue()));

            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, robotContainer, trajectories.getScoreCharge1Red());

        command.addCommands(follow(robotContainer, trajectories.getScoreCharge1Red()));

        return command;
    }

    public Command getScoreCharge2(RobotContainer robotContainer) {
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();

            resetRobotPose(command, robotContainer, trajectories.getScoreCharge2Blue());

            command.addCommands(follow(robotContainer, trajectories.getScoreCharge2Blue()));

            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, robotContainer, trajectories.getScoreCharge2Red());

        command.addCommands(follow(robotContainer, trajectories.getScoreCharge2Red()));

        return command;
    }

    public Command getScoreTwiceCharge0(RobotContainer robotContainer) {
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();

            resetRobotPose(command, robotContainer, trajectories.getScoreTwiceCharge0Blue());

            command.addCommands(follow(robotContainer, trajectories.getScoreTwiceCharge0Blue()));

            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, robotContainer, trajectories.getScoreTwiceCharge0Red());

        command.addCommands(follow(robotContainer, trajectories.getScoreTwiceCharge0Red()));

        return command;
    }

    public Command getScoreTwiceCharge1(RobotContainer robotContainer) {
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();

            resetRobotPose(command, robotContainer, trajectories.getScoreTwiceCharge1Blue());

            command.addCommands(follow(robotContainer, trajectories.getScoreTwiceCharge1Blue()));

            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, robotContainer, trajectories.getScoreTwiceCharge1Red());

        command.addCommands(follow(robotContainer, trajectories.getScoreTwiceCharge1Red()));

        return command;
    }

    public Command getScoreTwiceCharge2(RobotContainer robotContainer) {
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();

            resetRobotPose(command, robotContainer, trajectories.getScoreTwiceCharge2Blue());

            command.addCommands(follow(robotContainer, trajectories.getScoreTwiceCharge2Blue()));

            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, robotContainer, trajectories.getScoreTwiceCharge2Red());

        command.addCommands(follow(robotContainer, trajectories.getScoreTwiceCharge2Red()));

        return command;
    }

    public Command getScoreThreeCharge0(RobotContainer robotContainer) {
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();

            resetRobotPose(command, robotContainer, trajectories.getScoreThreeCharge0Blue());

            command.addCommands(follow(robotContainer, trajectories.getScoreThreeCharge0Blue()));

            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, robotContainer, trajectories.getScoreThreeCharge0Red());

        command.addCommands(follow(robotContainer, trajectories.getScoreThreeCharge0Red()));

        return command;
    }

    public Command getScoreThreeCharge1(RobotContainer robotContainer) {
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();

            resetRobotPose(command, robotContainer, trajectories.getScoreThreeCharge1Blue());

            command.addCommands(follow(robotContainer, trajectories.getScoreThreeCharge1Blue()));

            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, robotContainer, trajectories.getScoreThreeCharge1Red());

        command.addCommands(follow(robotContainer, trajectories.getScoreThreeCharge1Red()));

        return command;
    }
    
    public Command getScoreThreeCharge2(RobotContainer robotContainer) {
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();

            resetRobotPose(command, robotContainer, trajectories.getScoreThreeCharge2Blue());

            command.addCommands(follow(robotContainer, trajectories.getScoreThreeCharge2Blue()));

            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, robotContainer, trajectories.getScoreThreeCharge2Red());

        command.addCommands(follow(robotContainer, trajectories.getScoreThreeCharge2Red()));

        return command;
    }

    private Command follow(RobotContainer robotContainer, Trajectory trajectory) {
        return new FollowTrajectoryCommand(robotContainer.getDrivetrain(), trajectory);
    }

    public void resetRobotPose(SequentialCommandGroup command, RobotContainer robotContainer, Trajectory trajectory) {
        command.addCommands(new InstantCommand(() -> robotContainer.getDrivetrain().setPose(new Pose2d(robotContainer.s_Photon.getX(),
            robotContainer.s_Photon.getY(), new Rotation2d(robotContainer.s_Photon.getAngle())))));
    }

    public Command getCommand(RobotContainer robotContainer) {
        switch (autonomousModeChooser.getSelected()) {
            case DO_NOTHING :
            return getDoNothing(robotContainer);
                
            case LEAVE_COMMUNITY_0 :
            return getLeaveCommunity0(robotContainer);

            case LEAVE_COMMUNITY_1 :
            return getLeaveCommunity1(robotContainer);

            case LEAVE_COMMUNITY_2 :
            return getLeaveCommunity2(robotContainer);

            case SCORE_LEAVE_0 :
            return getScoreLeave0(robotContainer);

            case SCORE_LEAVE_1 :
            return getScoreLeave1(robotContainer);

            case SCORE_LEAVE_2 :
            return getScoreLeave2(robotContainer);

            case SCORE_TWICE_0 :
            return getScoreTwice0(robotContainer);

            case SCORE_TWICE_1 :
            return getScoreTwice1(robotContainer);

            case SCORE_TWICE_2 :
            return getScoreTwice2(robotContainer);

            case SCORE_THREE_0 :
            return getScoreThree0(robotContainer);

            case SCORE_THREE_1 :
            return getScoreThree1(robotContainer);

            case SCORE_THREE_2 :
            return getScoreThree2(robotContainer);

            case LEAVE_CHARGE_0 :
            return getLeaveCharge0(robotContainer);

            case LEAVE_CHARGE_1 :
            return getLeaveCharge1(robotContainer);

            case LEAVE_CHARGE_2 :
            return getLeaveCharge2(robotContainer);

            case SCORE_CHARGE_0 :
            return getScoreCharge0(robotContainer);

            case SCORE_CHARGE_1 :
            return getScoreCharge1(robotContainer);

            case SCORE_CHARGE_2 :
            return getScoreCharge2(robotContainer);

            case SCORE_TWICE_CHARGE_0 :
            return getScoreTwiceCharge0(robotContainer);

            case SCORE_TWICE_CHARGE_1 :
            return getScoreTwiceCharge1(robotContainer);

            case SCORE_TWICE_CHARGE_2 :
            return getScoreTwiceCharge2(robotContainer);

            case SCORE_THREE_CHARGE_0 :
            return getScoreThreeCharge0(robotContainer);

            case SCORE_THREE_CHARGE_1 :
            return getScoreThreeCharge1(robotContainer);

            case SCORE_THREE_CHARGE_2 :
            return getScoreThreeCharge2(robotContainer);
        }
        return new InstantCommand();
    }
    
    private enum AutonomousMode {
        DO_NOTHING, LEAVE_COMMUNITY_0, LEAVE_COMMUNITY_1, LEAVE_COMMUNITY_2, 
        SCORE_LEAVE_0, SCORE_LEAVE_1, SCORE_LEAVE_2, 
        SCORE_TWICE_0, SCORE_TWICE_1, SCORE_TWICE_2,
        SCORE_THREE_0, SCORE_THREE_1, SCORE_THREE_2,
        LEAVE_CHARGE_0, LEAVE_CHARGE_1, LEAVE_CHARGE_2,
        SCORE_CHARGE_0, SCORE_CHARGE_1, SCORE_CHARGE_2,
        SCORE_TWICE_CHARGE_0, SCORE_TWICE_CHARGE_1, SCORE_TWICE_CHARGE_2,
        SCORE_THREE_CHARGE_0, SCORE_THREE_CHARGE_1, SCORE_THREE_CHARGE_2
    }
}