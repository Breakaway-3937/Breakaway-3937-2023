package frc.robot.autos;

import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.RunArmAuto;
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
        autonomousModeChooser.addOption("Score Twice Mid", AutonomousMode.SCORE_TWICE_1);
        autonomousModeChooser.addOption("Score Twice Left", AutonomousMode.SCORE_TWICE_2);
        autonomousModeChooser.addOption("Score Three Right", AutonomousMode.SCORE_THREE_0);
        autonomousModeChooser.addOption("Score Three Mid", AutonomousMode.SCORE_THREE_1);
        autonomousModeChooser.addOption("Score Three Left", AutonomousMode.SCORE_THREE_2);
        autonomousModeChooser.addOption("Leave Charge Right", AutonomousMode.LEAVE_CHARGE_0);
        autonomousModeChooser.addOption("Leave Charge Mid", AutonomousMode.LEAVE_CHARGE_1);
        autonomousModeChooser.addOption("Leave Charge Left", AutonomousMode.LEAVE_CHARGE_2);
        autonomousModeChooser.addOption("Score Charge Right", AutonomousMode.SCORE_CHARGE_0);
        autonomousModeChooser.addOption("Score Charge Mid", AutonomousMode.SCORE_CHARGE_1);
        autonomousModeChooser.addOption("Score Charge Left", AutonomousMode.SCORE_CHARGE_2);
        autonomousModeChooser.addOption("Score Twice Charge Right", AutonomousMode.SCORE_TWICE_CHARGE_0);
        autonomousModeChooser.addOption("Score Twice Charge Mid", AutonomousMode.SCORE_TWICE_CHARGE_1);
        autonomousModeChooser.addOption("Score Twice Charge Left", AutonomousMode.SCORE_TWICE_CHARGE_2);
        autonomousModeChooser.addOption("Score Three Charge Right", AutonomousMode.SCORE_THREE_CHARGE_0);
        autonomousModeChooser.addOption("Score Three Charge Mid", AutonomousMode.SCORE_THREE_CHARGE_1);
        autonomousModeChooser.addOption("Score Three Charge Left", AutonomousMode.SCORE_THREE_CHARGE_2);
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
            new RunArmAuto(s_Arm, 2),
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
            new RunArmAuto(s_Arm, 2),
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
            new RunArmAuto(s_Arm, 2),
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

        SequentialCommandGroup command = new SequentialCommandGroup();
            command.addCommands(
            new InstantCommand(() -> s_Intake.setCone()),
            new RunArmAuto(s_Arm, 2),
            new SpitIntakeAuto(s_Intake),
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreTwice0().getInitialHolonomicPose())),
            new ParallelCommandGroup(new RunArmAuto(s_Arm, 0), swerveCommand));
        return command;
    }

    public Command getScoreTwice1() {
        var thetaController = new PIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand swerveCommand = new PPSwerveControllerCommand(
            trajectories.getScoreTwice1(), 
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
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreTwice1().getInitialHolonomicPose())),
            new ParallelCommandGroup(new RunArmAuto(s_Arm, 0), swerveCommand));
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

        SequentialCommandGroup command = new SequentialCommandGroup();
            command.addCommands(
            new InstantCommand(() -> s_Intake.setCone()),
            new RunArmAuto(s_Arm, 2),
            new SpitIntakeAuto(s_Intake),
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreTwice2().getInitialHolonomicPose())),
            new ParallelCommandGroup(new RunArmAuto(s_Arm, 0), swerveCommand));
        return command;
    }
    
    public Command getScoreThree0() {
        var thetaController = new PIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand swerveCommand = new PPSwerveControllerCommand(
            trajectories.getScoreThree0(), 
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
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreThree0().getInitialHolonomicPose())),
            new ParallelCommandGroup(new RunArmAuto(s_Arm, 0), swerveCommand));
        return command;
    }

    public Command getScoreThree1() {
        var thetaController = new PIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand swerveCommand = new PPSwerveControllerCommand(
            trajectories.getScoreThree1(), 
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
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreThree1().getInitialHolonomicPose())),
            new ParallelCommandGroup(new RunArmAuto(s_Arm, 0), swerveCommand));
        return command;
    }

    public Command getScoreThree2() {
        var thetaController = new PIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand swerveCommand = new PPSwerveControllerCommand(
            trajectories.getScoreThree2(), 
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
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreThree2().getInitialHolonomicPose())),
            new ParallelCommandGroup(new RunArmAuto(s_Arm, 0), swerveCommand));
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
            new RunArmAuto(s_Arm, 2),
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

    public Command getScoreTwiceCharge0() {
        var thetaController = new PIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand swerveCommand = new PPSwerveControllerCommand(
            trajectories.getScoreTwiceCharge0(), 
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
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreTwiceCharge0().getInitialHolonomicPose())),
            new ParallelCommandGroup(new RunArmAuto(s_Arm, 0), swerveCommand));
        return command;
    }

    public Command getScoreTwiceCharge1() {
        var thetaController = new PIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand swerveCommand = new PPSwerveControllerCommand(
            trajectories.getScoreTwiceCharge1(), 
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
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreTwiceCharge1().getInitialHolonomicPose())),
            new ParallelCommandGroup(new RunArmAuto(s_Arm, 0), swerveCommand));
        return command;
    }

    public Command getScoreTwiceCharge2() {
        var thetaController = new PIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand swerveCommand = new PPSwerveControllerCommand(
            trajectories.getScoreTwiceCharge2(), 
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
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreTwiceCharge2().getInitialHolonomicPose())),
            new ParallelCommandGroup(new RunArmAuto(s_Arm, 0), swerveCommand));
        return command;
    }

    public Command getScoreThreeCharge0() {
        var thetaController = new PIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand swerveCommand = new PPSwerveControllerCommand(
            trajectories.getScoreThreeCharge0(), 
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
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreThreeCharge0().getInitialHolonomicPose())),
            new ParallelCommandGroup(new RunArmAuto(s_Arm, 0), swerveCommand));
        return command;
    }

    public Command getScoreThreeCharge1() {
        var thetaController = new PIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand swerveCommand = new PPSwerveControllerCommand(
            trajectories.getScoreThreeCharge1(), 
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
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreThreeCharge1().getInitialHolonomicPose())),
            new ParallelCommandGroup(new RunArmAuto(s_Arm, 0), swerveCommand));
        return command;
    }
    
    public Command getScoreThreeCharge2() {
        var thetaController = new PIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand swerveCommand = new PPSwerveControllerCommand(
            trajectories.getScoreThreeCharge2(), 
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
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreThreeCharge2().getInitialHolonomicPose())),
            new ParallelCommandGroup(new RunArmAuto(s_Arm, 0), swerveCommand));
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

            case SCORE_TWICE_1 :
            return getScoreTwice1();

            case SCORE_TWICE_2 :
            return getScoreTwice2();

            case SCORE_THREE_0 :
            return getScoreThree0();

            case SCORE_THREE_1 :
            return getScoreThree1();

            case SCORE_THREE_2 :
            return getScoreThree2();

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

            case SCORE_TWICE_CHARGE_0 :
            return getScoreTwiceCharge0();

            case SCORE_TWICE_CHARGE_1 :
            return getScoreTwiceCharge1();

            case SCORE_TWICE_CHARGE_2 :
            return getScoreTwiceCharge2();

            case SCORE_THREE_CHARGE_0 :
            return getScoreThreeCharge0();

            case SCORE_THREE_CHARGE_1 :
            return getScoreThreeCharge1();

            case SCORE_THREE_CHARGE_2 :
            return getScoreThreeCharge2();
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