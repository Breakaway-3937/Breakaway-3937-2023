package frc.robot.autos;

import java.util.HashMap;

import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.RunArmAuto;
import frc.robot.commands.RunIntakeAuto;
import frc.robot.commands.SpitIntakeAuto;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class AutoChooser {
    private final AutoTrajectories trajectories;

    private final Drivetrain s_Drivetrain;
    private final Arm s_Arm;
    private final Intake s_Intake;

    private final SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();

    public AutoChooser(AutoTrajectories trajectories, Drivetrain s_Drivetrain, Arm s_Arm, Intake s_Intake) {
        this.s_Drivetrain = s_Drivetrain;
        this.trajectories = trajectories;
        this.s_Arm = s_Arm;
        this.s_Intake = s_Intake;
        autonomousModeChooser.setDefaultOption("Do Nothing", AutonomousMode.DO_NOTHING);
        autonomousModeChooser.addOption("Leave Community Right", AutonomousMode.LEAVE_COMMUNITY_0);
        autonomousModeChooser.addOption("Leave Community Mid", AutonomousMode.LEAVE_COMMUNITY_1);
        autonomousModeChooser.addOption("Leave Community Left", AutonomousMode.LEAVE_COMMUNITY_2);
        autonomousModeChooser.addOption("Leave Charge Right", AutonomousMode.LEAVE_CHARGE_0);
        autonomousModeChooser.addOption("Leave Charge Mid", AutonomousMode.LEAVE_CHARGE_1);
        autonomousModeChooser.addOption("Leave Charge Left", AutonomousMode.LEAVE_CHARGE_2);
        autonomousModeChooser.addOption("Score Leave Right", AutonomousMode.SCORE_LEAVE_0);
        autonomousModeChooser.addOption("Score Leave Mid", AutonomousMode.SCORE_LEAVE_1);
        autonomousModeChooser.addOption("Score Leave Left", AutonomousMode.SCORE_LEAVE_2);
        autonomousModeChooser.addOption("Score Twice Right", AutonomousMode.SCORE_TWICE_0);
        autonomousModeChooser.addOption("Score Twice Left", AutonomousMode.SCORE_TWICE_2);
        autonomousModeChooser.addOption("Leave Charge Right", AutonomousMode.LEAVE_CHARGE_0);
        autonomousModeChooser.addOption("Leave Charge Mid", AutonomousMode.LEAVE_CHARGE_1);
        autonomousModeChooser.addOption("Leave Charge Left", AutonomousMode.LEAVE_CHARGE_2);
        autonomousModeChooser.addOption("Score Charge Right", AutonomousMode.SCORE_CHARGE_0);
        autonomousModeChooser.addOption("Score Charge Mid", AutonomousMode.SCORE_CHARGE_1);
        autonomousModeChooser.addOption("Score Charge Left", AutonomousMode.SCORE_CHARGE_2);
        autonomousModeChooser.addOption("Score Grab Charge Right", AutonomousMode.SCORE_GRAB_CHARGE_0);
        autonomousModeChooser.addOption("Score Grab Charge Mid", AutonomousMode.SCORE_GRAB_CHARGE_1);
        autonomousModeChooser.addOption("Score Grab Charge Left", AutonomousMode.SCORE_GRAB_CHARGE_2);
    }

    public SendableChooser<AutonomousMode> getModeChooser() {
        return autonomousModeChooser;
    }

    public Command getDoNothing() {
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(new RunArmAuto(s_Arm, 0));
        return command;
    }

    public Command getLeaveCommunity0() {
        var thetaController = new PIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand swerveCommand = new PPSwerveControllerCommand(
            trajectories.getLeaveCommunity0(), 
            s_Drivetrain::getPose, 
            Constants.Drivetrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates, 
            false,
            s_Drivetrain);
        SequentialCommandGroup command = new SequentialCommandGroup();
            command.addCommands(
            new RunArmAuto(s_Arm, 0),
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getLeaveCommunity0().getInitialHolonomicPose())),
            new ParallelCommandGroup(new RunArmAuto(s_Arm, 0), swerveCommand));
        return command;
        
    }

    public Command getLeaveCommunity1() {
        var thetaController = new PIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand swerveCommand = new PPSwerveControllerCommand(
            trajectories.getLeaveCommunity1(), 
            s_Drivetrain::getPose, 
            Constants.Drivetrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates, 
            false,
            s_Drivetrain);
        SequentialCommandGroup command = new SequentialCommandGroup();
            command.addCommands(
            new RunArmAuto(s_Arm, 0),
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getLeaveCommunity1().getInitialHolonomicPose())),
            new ParallelCommandGroup(new RunArmAuto(s_Arm, 0), swerveCommand));
        return command;
    }

    public Command getLeaveCommunity2() {
        var thetaController = new PIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand swerveCommand = new PPSwerveControllerCommand(
            trajectories.getLeaveCommunity2(), 
            s_Drivetrain::getPose, 
            Constants.Drivetrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates, 
            false,
            s_Drivetrain);
        SequentialCommandGroup command = new SequentialCommandGroup();
            command.addCommands(
            new RunArmAuto(s_Arm, 0),
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getLeaveCommunity2().getInitialHolonomicPose())),
            new ParallelCommandGroup(new RunArmAuto(s_Arm, 0), swerveCommand));
        return command;
    }

    public Command getScoreLeave0() {
        var thetaController = new PIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand swerveCommand = new PPSwerveControllerCommand(
            trajectories.getScoreLeave0(), 
            s_Drivetrain::getPose, 
            Constants.Drivetrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates, 
            false,
            s_Drivetrain);
        SequentialCommandGroup command = new SequentialCommandGroup();
            command.addCommands(
            new InstantCommand(() -> s_Intake.setCone()),
            new RunArmAuto(s_Arm, 3),
            new SpitIntakeAuto(s_Intake),
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreLeave0().getInitialHolonomicPose())),
            new ParallelCommandGroup(new RunArmAuto(s_Arm, 0), swerveCommand));
        return command;
    }

    public Command getScoreLeave1() {
        var thetaController = new PIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand swerveCommand = new PPSwerveControllerCommand(
            trajectories.getScoreLeave1(), 
            s_Drivetrain::getPose, 
            Constants.Drivetrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates, 
            false,
            s_Drivetrain);
        SequentialCommandGroup command = new SequentialCommandGroup();
            command.addCommands(
            new InstantCommand(() -> s_Intake.setCone()),
            new RunArmAuto(s_Arm, 3),
            new SpitIntakeAuto(s_Intake),
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreLeave1().getInitialHolonomicPose())),
            new ParallelCommandGroup(new RunArmAuto(s_Arm, 0), swerveCommand));
        return command;
    }

    public Command getScoreLeave2() {
        var thetaController = new PIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand swerveCommand = new PPSwerveControllerCommand(
            trajectories.getScoreLeave2(), 
            s_Drivetrain::getPose, 
            Constants.Drivetrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates, 
            false,
            s_Drivetrain);
        SequentialCommandGroup command = new SequentialCommandGroup();
            command.addCommands(
            new InstantCommand(() -> s_Intake.setCone()),
            new RunArmAuto(s_Arm, 3),
            new SpitIntakeAuto(s_Intake),
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreLeave2().getInitialHolonomicPose())),
            new ParallelCommandGroup(new RunArmAuto(s_Arm, 0), swerveCommand));
        return command;
    }

    public Command getScoreTwice0() {
        var thetaController = new PIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand swerveCommand = new PPSwerveControllerCommand(
            trajectories.getScoreTwice0(), 
            s_Drivetrain::getPose, 
            Constants.Drivetrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates, 
            false,
            s_Drivetrain);

        PPSwerveControllerCommand swerveCommand1 = new PPSwerveControllerCommand(
            trajectories.getScoreTwice0Leave(), 
            s_Drivetrain::getPose, 
            Constants.Drivetrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates, 
            false,
            s_Drivetrain);

        HashMap<String, Command> eventMap = new HashMap<>();
            eventMap.put("intake", new SequentialCommandGroup(new InstantCommand(() -> s_Intake.setCube()), new RunArmAuto(s_Arm, -1), new ParallelRaceGroup(new WaitCommand(2.25), new RunIntakeAuto(s_Intake)), new RunArmAuto(s_Arm, 0)));
            

        FollowPathWithEvents followCommand = new FollowPathWithEvents(
            swerveCommand,
            trajectories.getScoreTwice0().getMarkers(),
            eventMap
        );

        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(
        new InstantCommand(() -> s_Intake.setCone()),
        new RunArmAuto(s_Arm, 3),
        new SpitIntakeAuto(s_Intake),
        new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreTwice0().getInitialHolonomicPose())),
        new SequentialCommandGroup(new RunArmAuto(s_Arm, 0), followCommand),
        new RunArmAuto(s_Arm, 3),
        new SpitIntakeAuto(s_Intake),
        new RunArmAuto(s_Arm, 0),
        new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreTwice0Leave().getInitialHolonomicPose())),
        swerveCommand1);
        return command;
    }

    public Command getScoreTwice2() {
        var thetaController = new PIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand swerveCommand = new PPSwerveControllerCommand(
            trajectories.getScoreTwice2(), 
            s_Drivetrain::getPose, 
            Constants.Drivetrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates, 
            false,
            s_Drivetrain);

        PPSwerveControllerCommand swerveCommand1 = new PPSwerveControllerCommand(
            trajectories.getScoreTwice2Leave(), 
            s_Drivetrain::getPose, 
            Constants.Drivetrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates, 
            false,
            s_Drivetrain);

        HashMap<String, Command> eventMap = new HashMap<>();
            eventMap.put("intake", new SequentialCommandGroup(new InstantCommand(() -> s_Intake.setCube()), new RunArmAuto(s_Arm, -1), new ParallelRaceGroup(new RunIntakeAuto(s_Intake), new WaitCommand(2.25)), new RunArmAuto(s_Arm, 0)));
            

        FollowPathWithEvents followCommand = new FollowPathWithEvents(
            swerveCommand,
            trajectories.getScoreTwice2().getMarkers(),
            eventMap
        );

        SequentialCommandGroup command = new SequentialCommandGroup();
            command.addCommands(
            new InstantCommand(() -> s_Intake.setCone()),
            new RunArmAuto(s_Arm, 3),
            new SpitIntakeAuto(s_Intake),
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreTwice2().getInitialHolonomicPose())),
            new SequentialCommandGroup(new RunArmAuto(s_Arm, 0), followCommand),
            new RunArmAuto(s_Arm, 3),
            new SpitIntakeAuto(s_Intake),
            new RunArmAuto(s_Arm, 0),
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreTwice2Leave().getInitialHolonomicPose())),
            swerveCommand1);
        return command;
    }

    public Command getLeaveCharge0() {
        var thetaController = new PIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand swerveCommand = new PPSwerveControllerCommand(
            trajectories.getLeaveCharge0(), 
            s_Drivetrain::getPose, 
            Constants.Drivetrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates, 
            false,
            s_Drivetrain);
        SequentialCommandGroup command = new SequentialCommandGroup();
            command.addCommands(
            new RunArmAuto(s_Arm, 0),
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getLeaveCharge0().getInitialHolonomicPose())),
            new ParallelCommandGroup(new RunArmAuto(s_Arm, 0), swerveCommand),
            new AutoBalance(s_Drivetrain));
        return command;
    }

    public Command getLeaveCharge1() {
        var thetaController = new PIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand swerveCommand = new PPSwerveControllerCommand(
            trajectories.getLeaveCharge1(), 
            s_Drivetrain::getPose, 
            Constants.Drivetrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates, 
            false,
            s_Drivetrain);
        SequentialCommandGroup command = new SequentialCommandGroup();
            command.addCommands(
            new RunArmAuto(s_Arm, 0),
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getLeaveCharge1().getInitialHolonomicPose())),
            new ParallelCommandGroup(new RunArmAuto(s_Arm, 0), swerveCommand),
            new AutoBalance(s_Drivetrain));
        return command;
    }

    public Command getLeaveCharge2() {
        var thetaController = new PIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand swerveCommand = new PPSwerveControllerCommand(
            trajectories.getLeaveCharge2(), 
            s_Drivetrain::getPose, 
            Constants.Drivetrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates, 
            false,
            s_Drivetrain);
        SequentialCommandGroup command = new SequentialCommandGroup();
            command.addCommands(
            new RunArmAuto(s_Arm, 0),
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getLeaveCharge2().getInitialHolonomicPose())),
            new ParallelCommandGroup(new RunArmAuto(s_Arm, 0), swerveCommand),
            new AutoBalance(s_Drivetrain));
        return command;
    }

    public Command getScoreCharge0() {
        var thetaController = new PIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand swerveCommand = new PPSwerveControllerCommand(
            trajectories.getScoreCharge0(), 
            s_Drivetrain::getPose, 
            Constants.Drivetrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates, 
            false,
            s_Drivetrain);
        SequentialCommandGroup command = new SequentialCommandGroup();
            command.addCommands(
            new InstantCommand(() -> s_Intake.setCone()),
            new RunArmAuto(s_Arm, 2),
            new SpitIntakeAuto(s_Intake),
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreCharge0().getInitialHolonomicPose())),
            new ParallelCommandGroup(new RunArmAuto(s_Arm, 0), swerveCommand),
            new AutoBalance(s_Drivetrain));
        return command;
    }

    public Command getScoreCharge1() {
        var thetaController = new PIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand swerveCommand = new PPSwerveControllerCommand(
            trajectories.getScoreCharge1(), 
            s_Drivetrain::getPose, 
            Constants.Drivetrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates, 
            false,
            s_Drivetrain);
        SequentialCommandGroup command = new SequentialCommandGroup();
            command.addCommands(
            new InstantCommand(() -> s_Intake.setCone()),
            new RunArmAuto(s_Arm, 3),
            new SpitIntakeAuto(s_Intake),
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreCharge1().getInitialHolonomicPose())),
            new ParallelCommandGroup(new RunArmAuto(s_Arm, 0), swerveCommand),
            new AutoBalance(s_Drivetrain));
        return command;
    }

    public Command getScoreCharge2() {
        var thetaController = new PIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand swerveCommand = new PPSwerveControllerCommand(
            trajectories.getScoreCharge2(), 
            s_Drivetrain::getPose, 
            Constants.Drivetrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates, 
            false,
            s_Drivetrain);
        SequentialCommandGroup command = new SequentialCommandGroup();
            command.addCommands(
            new InstantCommand(() -> s_Intake.setCone()),
            new RunArmAuto(s_Arm, 2),
            new SpitIntakeAuto(s_Intake),
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreCharge2().getInitialHolonomicPose())),
            new ParallelCommandGroup(new RunArmAuto(s_Arm, 0), swerveCommand),
            new AutoBalance(s_Drivetrain));
        return command;
    }

    public Command getScoreGrabCharge0() {
        var thetaController = new PIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand swerveCommand = new PPSwerveControllerCommand(
            trajectories.getScoreGrabCharge0(), 
            s_Drivetrain::getPose, 
            Constants.Drivetrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates, 
            false,
            s_Drivetrain);

        HashMap<String, Command> eventMap = new HashMap<>();
            eventMap.put("intake", new SequentialCommandGroup(new InstantCommand(() -> s_Intake.setCube()), new RunArmAuto(s_Arm, -1), new ParallelRaceGroup(new WaitCommand(2.25), new RunIntakeAuto(s_Intake)), new RunArmAuto(s_Arm, 0)));
            

        FollowPathWithEvents followCommand = new FollowPathWithEvents(
            swerveCommand,
            trajectories.getScoreGrabCharge0().getMarkers(),
            eventMap
        );

        SequentialCommandGroup command = new SequentialCommandGroup();
            command.addCommands(
            new InstantCommand(() -> s_Intake.setCone()),
            new RunArmAuto(s_Arm, 3),
            new SpitIntakeAuto(s_Intake),
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreGrabCharge0().getInitialHolonomicPose())),
            new SequentialCommandGroup(new RunArmAuto(s_Arm, 0), followCommand),
            new AutoBalance(s_Drivetrain));
        return command;
    }

    public Command getScoreGrabCharge1() {
        var thetaController = new PIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand swerveCommand = new PPSwerveControllerCommand(
            trajectories.getScoreGrabCharge1(), 
            s_Drivetrain::getPose, 
            Constants.Drivetrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates, 
            false,
            s_Drivetrain);

        HashMap<String, Command> eventMap = new HashMap<>();
            eventMap.put("intake", new SequentialCommandGroup(new InstantCommand(() -> s_Intake.setCube()), new RunArmAuto(s_Arm, -1), new ParallelRaceGroup(new WaitCommand(2.25), new RunIntakeAuto(s_Intake)), new RunArmAuto(s_Arm, 0)));
            

        FollowPathWithEvents followCommand = new FollowPathWithEvents(
            swerveCommand,
            trajectories.getScoreGrabCharge1().getMarkers(),
            eventMap
        );

        SequentialCommandGroup command = new SequentialCommandGroup();
            command.addCommands(
            new InstantCommand(() -> s_Intake.setCone()),
            new RunArmAuto(s_Arm, 3),
            new SpitIntakeAuto(s_Intake),
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreGrabCharge1().getInitialHolonomicPose())),
            new SequentialCommandGroup(new RunArmAuto(s_Arm, 0), followCommand),
            new AutoBalance(s_Drivetrain));
        return command;
    }

    public Command getScoreGrabCharge2() {
        var thetaController = new PIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand swerveCommand = new PPSwerveControllerCommand(
            trajectories.getScoreGrabCharge2(), 
            s_Drivetrain::getPose, 
            Constants.Drivetrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates, 
            false,
            s_Drivetrain);

        HashMap<String, Command> eventMap = new HashMap<>();
            eventMap.put("intake", new SequentialCommandGroup(new InstantCommand(() -> s_Intake.setCube()), new RunArmAuto(s_Arm, -1), new ParallelRaceGroup(new WaitCommand(2.25), new RunIntakeAuto(s_Intake)), new RunArmAuto(s_Arm, 0)));
            

        FollowPathWithEvents followCommand = new FollowPathWithEvents(
            swerveCommand,
            trajectories.getScoreGrabCharge2().getMarkers(),
            eventMap
        );

        SequentialCommandGroup command = new SequentialCommandGroup();
            command.addCommands(
            new InstantCommand(() -> s_Intake.setCone()),
            new RunArmAuto(s_Arm, 3),
            new SpitIntakeAuto(s_Intake),
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreGrabCharge2().getInitialHolonomicPose())),
            new SequentialCommandGroup(new RunArmAuto(s_Arm, 0), followCommand),
            new AutoBalance(s_Drivetrain));
        return command;
    }

    public Command getCommand() {
        switch (autonomousModeChooser.getSelected()) {
            case DO_NOTHING :
            return getDoNothing();
                
            case LEAVE_COMMUNITY_0 :
            return getLeaveCommunity0();

            case LEAVE_COMMUNITY_1 :
            return getLeaveCommunity1();

            case LEAVE_COMMUNITY_2 :
            return getLeaveCommunity2();

            case SCORE_LEAVE_0 :
            return getScoreLeave0();

            case SCORE_LEAVE_1 :
            return getScoreLeave1();

            case SCORE_LEAVE_2 :
            return getScoreLeave2();

            case SCORE_TWICE_0 :
            return getScoreTwice0();

            case SCORE_TWICE_2 :
            return getScoreTwice2();

            case LEAVE_CHARGE_0 :
            return getLeaveCharge0();

            case LEAVE_CHARGE_1 :
            return getLeaveCharge1();

            case LEAVE_CHARGE_2 :
            return getLeaveCharge2();

            case SCORE_CHARGE_0 :
            return getScoreCharge0();

            case SCORE_CHARGE_1 :
            return getScoreCharge1();

            case SCORE_CHARGE_2 :
            return getScoreCharge2();

            case SCORE_GRAB_CHARGE_0 :
            return getScoreGrabCharge0();

            case SCORE_GRAB_CHARGE_1 :
            return getScoreGrabCharge1();

            case SCORE_GRAB_CHARGE_2 :
            return getScoreGrabCharge2();
        }
        return new InstantCommand();
    }
    
    private enum AutonomousMode {
        DO_NOTHING, LEAVE_COMMUNITY_0, LEAVE_COMMUNITY_1, LEAVE_COMMUNITY_2, 
        SCORE_LEAVE_0, SCORE_LEAVE_1, SCORE_LEAVE_2, 
        SCORE_TWICE_0, SCORE_TWICE_2,
        LEAVE_CHARGE_0, LEAVE_CHARGE_1, LEAVE_CHARGE_2,
        SCORE_CHARGE_0, SCORE_CHARGE_1, SCORE_CHARGE_2,
        SCORE_GRAB_CHARGE_0, SCORE_GRAB_CHARGE_1, SCORE_GRAB_CHARGE_2
    }
}