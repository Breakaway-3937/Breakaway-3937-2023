package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class AutoChooser {
    private final AutoTrajectories trajectories;

    private final DriveTrain s_Drivetrain;

    private final SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();

    public AutoChooser(AutoTrajectories trajectories, DriveTrain s_Drivetrain) {
        this.s_Drivetrain = s_Drivetrain;
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

    public Command getDoNothing() {
        SequentialCommandGroup command = new SequentialCommandGroup();
        return command;
    }

    public Command getLeaveCommunity0() {
        var thetaController = new ProfiledPIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0, Constants.Auto.KTHETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        SequentialCommandGroup command = new SequentialCommandGroup();
        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectories.getLeaveCommunity0Blue(),
            s_Drivetrain::getPose,
            Constants.DriveTrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates,
            s_Drivetrain);

        command.addCommands(
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getLeaveCommunity0Blue().getInitialPose())),
            swerveControllerCommand);

        return command;
        

        /*SequentialCommandGroup command = new SequentialCommandGroup();
        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectories.getLeaveCommunity0Red(),
            s_Drivetrain::getPose,
            Constants.DriveTrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates,
            s_Drivetrain);

        command.addCommands(
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getLeaveCommunity0R().getInitialPose())),
            swerveControllerCommand);

        return command;*/
        
    }

    public Command getLeaveCommunity1() {
        var thetaController = new ProfiledPIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0, Constants.Auto.KTHETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();
            SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                trajectories.getLeaveCommunity1Blue(),
                s_Drivetrain::getPose,
                Constants.DriveTrain.SWERVE_KINEMATICS,
                new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
                new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
                thetaController,
                s_Drivetrain::setModuleStates,
                s_Drivetrain);
    
            command.addCommands(
                new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getLeaveCommunity1Blue().getInitialPose())),
                swerveControllerCommand);
        return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();
        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectories.getLeaveCommunity1Red(),
            s_Drivetrain::getPose,
            Constants.DriveTrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates,
            s_Drivetrain);

        command.addCommands(
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getLeaveCommunity1Red().getInitialPose())),
            swerveControllerCommand);
        return command;
    }

    public Command getLeaveCommunity2() {
        var thetaController = new ProfiledPIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0, Constants.Auto.KTHETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();
            SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                trajectories.getLeaveCommunity2Blue(),
                s_Drivetrain::getPose,
                Constants.DriveTrain.SWERVE_KINEMATICS,
                new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
                new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
                thetaController,
                s_Drivetrain::setModuleStates,
                s_Drivetrain);
    
            command.addCommands(
                new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getLeaveCommunity2Blue().getInitialPose())),
                swerveControllerCommand);
            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();
        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectories.getLeaveCommunity2Red(),
            s_Drivetrain::getPose,
            Constants.DriveTrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates,
            s_Drivetrain);

        command.addCommands(
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getLeaveCommunity2Red().getInitialPose())),
            swerveControllerCommand);
        return command;
    }

    public Command getScoreLeave0() {
        var thetaController = new ProfiledPIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0, Constants.Auto.KTHETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();
            SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                trajectories.getScoreLeave0Blue(),
                s_Drivetrain::getPose,
                Constants.DriveTrain.SWERVE_KINEMATICS,
                new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
                new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
                thetaController,
                s_Drivetrain::setModuleStates,
                s_Drivetrain);
    
            command.addCommands(
                new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreLeave0Blue().getInitialPose())),
                swerveControllerCommand);
            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();
        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectories.getScoreLeave0Red(),
            s_Drivetrain::getPose,
            Constants.DriveTrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates,
            s_Drivetrain);

        command.addCommands(
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreLeave0Red().getInitialPose())),
            swerveControllerCommand);
        return command;
    }

    public Command getScoreLeave1() {
        var thetaController = new ProfiledPIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0, Constants.Auto.KTHETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();
            SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                trajectories.getScoreLeave1Blue(),
                s_Drivetrain::getPose,
                Constants.DriveTrain.SWERVE_KINEMATICS,
                new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
                new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
                thetaController,
                s_Drivetrain::setModuleStates,
                s_Drivetrain);
    
            command.addCommands(
                new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreLeave1Blue().getInitialPose())),
                swerveControllerCommand);
            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();
        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectories.getScoreLeave1Red(),
            s_Drivetrain::getPose,
            Constants.DriveTrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates,
            s_Drivetrain);

        command.addCommands(
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreLeave1Red().getInitialPose())),
            swerveControllerCommand);
        return command;
    }

    public Command getScoreLeave2() {
        var thetaController = new ProfiledPIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0, Constants.Auto.KTHETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();
            SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                trajectories.getScoreLeave2Blue(),
                s_Drivetrain::getPose,
                Constants.DriveTrain.SWERVE_KINEMATICS,
                new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
                new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
                thetaController,
                s_Drivetrain::setModuleStates,
                s_Drivetrain);
    
            command.addCommands(
                new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreLeave2Blue().getInitialPose())),
                swerveControllerCommand);
            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();
        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectories.getScoreLeave2Red(),
            s_Drivetrain::getPose,
            Constants.DriveTrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates,
            s_Drivetrain);

        command.addCommands(
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreLeave2Red().getInitialPose())),
            swerveControllerCommand);
        return command;
    }

    public Command getScoreTwice0() {
        var thetaController = new ProfiledPIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0, Constants.Auto.KTHETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();
            SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                trajectories.getScoreTwice0Blue(),
                s_Drivetrain::getPose,
                Constants.DriveTrain.SWERVE_KINEMATICS,
                new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
                new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
                thetaController,
                s_Drivetrain::setModuleStates,
                s_Drivetrain);
    
            command.addCommands(
                new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreTwice0Blue().getInitialPose())),
                swerveControllerCommand);
            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();
        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectories.getScoreTwice0Red(),
            s_Drivetrain::getPose,
            Constants.DriveTrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates,
            s_Drivetrain);

        command.addCommands(
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreTwice0Red().getInitialPose())),
            swerveControllerCommand);
        return command;
    }

    public Command getScoreTwice1() {
        var thetaController = new ProfiledPIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0, Constants.Auto.KTHETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();
            SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                trajectories.getScoreTwice1Blue(),
                s_Drivetrain::getPose,
                Constants.DriveTrain.SWERVE_KINEMATICS,
                new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
                new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
                thetaController,
                s_Drivetrain::setModuleStates,
                s_Drivetrain);
    
            command.addCommands(
                new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreTwice1Blue().getInitialPose())),
                swerveControllerCommand);
            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();
        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectories.getScoreTwice1Red(),
            s_Drivetrain::getPose,
            Constants.DriveTrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates,
            s_Drivetrain);

        command.addCommands(
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreTwice1Red().getInitialPose())),
            swerveControllerCommand);
        return command;
    }

    public Command getScoreTwice2() {
        var thetaController = new ProfiledPIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0, Constants.Auto.KTHETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();
            SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                trajectories.getScoreTwice2Blue(),
                s_Drivetrain::getPose,
                Constants.DriveTrain.SWERVE_KINEMATICS,
                new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
                new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
                thetaController,
                s_Drivetrain::setModuleStates,
                s_Drivetrain);
    
            command.addCommands(
                new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreTwice2Blue().getInitialPose())),
                swerveControllerCommand);
            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();
        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectories.getScoreTwice2Red(),
            s_Drivetrain::getPose,
            Constants.DriveTrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates,
            s_Drivetrain);

        command.addCommands(
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreTwice2Red().getInitialPose())),
            swerveControllerCommand);
        return command;
    }
    
    public Command getScoreThree0() {
        var thetaController = new ProfiledPIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0, Constants.Auto.KTHETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();
            SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                trajectories.getScoreThree0Blue(),
                s_Drivetrain::getPose,
                Constants.DriveTrain.SWERVE_KINEMATICS,
                new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
                new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
                thetaController,
                s_Drivetrain::setModuleStates,
                s_Drivetrain);
    
            command.addCommands(
                new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreThree0Blue().getInitialPose())),
                swerveControllerCommand);
            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();
        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectories.getScoreThree0Red(),
            s_Drivetrain::getPose,
            Constants.DriveTrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates,
            s_Drivetrain);

        command.addCommands(
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreThree0Red().getInitialPose())),
            swerveControllerCommand);
        return command;
    }

    public Command getScoreThree1() {
        var thetaController = new ProfiledPIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0, Constants.Auto.KTHETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();
            SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                trajectories.getScoreThree1Blue(),
                s_Drivetrain::getPose,
                Constants.DriveTrain.SWERVE_KINEMATICS,
                new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
                new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
                thetaController,
                s_Drivetrain::setModuleStates,
                s_Drivetrain);
    
            command.addCommands(
                new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreThree1Blue().getInitialPose())),
                swerveControllerCommand);
            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();
        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectories.getScoreThree1Red(),
            s_Drivetrain::getPose,
            Constants.DriveTrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates,
            s_Drivetrain);

        command.addCommands(
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreThree1Red().getInitialPose())),
            swerveControllerCommand);
        return command;
    }

    public Command getScoreThree2() {
        var thetaController = new ProfiledPIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0, Constants.Auto.KTHETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();
            SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                trajectories.getScoreThree2Blue(),
                s_Drivetrain::getPose,
                Constants.DriveTrain.SWERVE_KINEMATICS,
                new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
                new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
                thetaController,
                s_Drivetrain::setModuleStates,
                s_Drivetrain);
    
            command.addCommands(
                new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreThree2Blue().getInitialPose())),
                swerveControllerCommand);
            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();
        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectories.getScoreThree2Red(),
            s_Drivetrain::getPose,
            Constants.DriveTrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates,
            s_Drivetrain);

        command.addCommands(
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreThree2Red().getInitialPose())),
            swerveControllerCommand);
        return command;
    }

    public Command getLeaveCharge0() {
        var thetaController = new ProfiledPIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0, Constants.Auto.KTHETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();
            SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                trajectories.getLeaveCharge0Blue(),
                s_Drivetrain::getPose,
                Constants.DriveTrain.SWERVE_KINEMATICS,
                new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
                new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
                thetaController,
                s_Drivetrain::setModuleStates,
                s_Drivetrain);
    
            command.addCommands(
                new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getLeaveCharge0Blue().getInitialPose())),
                swerveControllerCommand);
            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();
        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectories.getLeaveCharge0Red(),
            s_Drivetrain::getPose,
            Constants.DriveTrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates,
            s_Drivetrain);

        command.addCommands(
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getLeaveCharge0Red().getInitialPose())),
            swerveControllerCommand);
        return command;
    }

    public Command getLeaveCharge1() {
        var thetaController = new ProfiledPIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0, Constants.Auto.KTHETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();
            SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                trajectories.getLeaveCharge1Blue(),
                s_Drivetrain::getPose,
                Constants.DriveTrain.SWERVE_KINEMATICS,
                new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
                new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
                thetaController,
                s_Drivetrain::setModuleStates,
                s_Drivetrain);
    
            command.addCommands(
                new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getLeaveCharge1Blue().getInitialPose())),
                swerveControllerCommand);
            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();
        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectories.getLeaveCharge1Red(),
            s_Drivetrain::getPose,
            Constants.DriveTrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates,
            s_Drivetrain);

        command.addCommands(
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getLeaveCharge1Red().getInitialPose())),
            swerveControllerCommand);
        return command;
    }

    public Command getLeaveCharge2() {
        var thetaController = new ProfiledPIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0, Constants.Auto.KTHETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();
            SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                trajectories.getLeaveCharge2Blue(),
                s_Drivetrain::getPose,
                Constants.DriveTrain.SWERVE_KINEMATICS,
                new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
                new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
                thetaController,
                s_Drivetrain::setModuleStates,
                s_Drivetrain);
    
            command.addCommands(
                new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getLeaveCharge2Blue().getInitialPose())),
                swerveControllerCommand);
            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();
        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectories.getLeaveCharge2Red(),
            s_Drivetrain::getPose,
            Constants.DriveTrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates,
            s_Drivetrain);

        command.addCommands(
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getLeaveCharge2Red().getInitialPose())),
            swerveControllerCommand);
        return command;
    }

    public Command getScoreCharge0() {
        var thetaController = new ProfiledPIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0, Constants.Auto.KTHETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();
            SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                trajectories.getScoreCharge0Blue(),
                s_Drivetrain::getPose,
                Constants.DriveTrain.SWERVE_KINEMATICS,
                new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
                new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
                thetaController,
                s_Drivetrain::setModuleStates,
                s_Drivetrain);
    
            command.addCommands(
                new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreCharge0Blue().getInitialPose())),
                swerveControllerCommand);
            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();
        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectories.getScoreCharge0Red(),
            s_Drivetrain::getPose,
            Constants.DriveTrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates,
            s_Drivetrain);

        command.addCommands(
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreCharge0Red().getInitialPose())),
            swerveControllerCommand);
        return command;
    }

    public Command getScoreCharge1() {
        var thetaController = new ProfiledPIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0, Constants.Auto.KTHETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();
            SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                trajectories.getScoreCharge1Blue(),
                s_Drivetrain::getPose,
                Constants.DriveTrain.SWERVE_KINEMATICS,
                new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
                new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
                thetaController,
                s_Drivetrain::setModuleStates,
                s_Drivetrain);
    
            command.addCommands(
                new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreCharge1Blue().getInitialPose())),
                swerveControllerCommand);
            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();
        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectories.getScoreCharge1Red(),
            s_Drivetrain::getPose,
            Constants.DriveTrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates,
            s_Drivetrain);

        command.addCommands(
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreCharge1Red().getInitialPose())),
            swerveControllerCommand);
        return command;
    }

    public Command getScoreCharge2() {
        var thetaController = new ProfiledPIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0, Constants.Auto.KTHETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();
            SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                trajectories.getScoreCharge2Blue(),
                s_Drivetrain::getPose,
                Constants.DriveTrain.SWERVE_KINEMATICS,
                new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
                new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
                thetaController,
                s_Drivetrain::setModuleStates,
                s_Drivetrain);
    
            command.addCommands(
                new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreCharge2Blue().getInitialPose())),
                swerveControllerCommand);
            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();
        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectories.getScoreCharge2Red(),
            s_Drivetrain::getPose,
            Constants.DriveTrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates,
            s_Drivetrain);

        command.addCommands(
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreCharge2Red().getInitialPose())),
            swerveControllerCommand);
        return command;
    }

    public Command getScoreTwiceCharge0() {
        var thetaController = new ProfiledPIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0, Constants.Auto.KTHETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();
            SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                trajectories.getScoreTwiceCharge0Blue(),
                s_Drivetrain::getPose,
                Constants.DriveTrain.SWERVE_KINEMATICS,
                new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
                new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
                thetaController,
                s_Drivetrain::setModuleStates,
                s_Drivetrain);
    
            command.addCommands(
                new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreTwiceCharge0Blue().getInitialPose())),
                swerveControllerCommand);
            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();
        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectories.getScoreTwiceCharge0Red(),
            s_Drivetrain::getPose,
            Constants.DriveTrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates,
            s_Drivetrain);

        command.addCommands(
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreTwice0Red().getInitialPose())),
            swerveControllerCommand);
        return command;
    }

    public Command getScoreTwiceCharge1() {
        var thetaController = new ProfiledPIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0, Constants.Auto.KTHETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();
            SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                trajectories.getScoreTwiceCharge1Blue(),
                s_Drivetrain::getPose,
                Constants.DriveTrain.SWERVE_KINEMATICS,
                new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
                new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
                thetaController,
                s_Drivetrain::setModuleStates,
                s_Drivetrain);
    
            command.addCommands(
                new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreTwiceCharge1Blue().getInitialPose())),
                swerveControllerCommand);
            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();
        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectories.getScoreTwiceCharge1Red(),
            s_Drivetrain::getPose,
            Constants.DriveTrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates,
            s_Drivetrain);

        command.addCommands(
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreTwice1Red().getInitialPose())),
            swerveControllerCommand);
        return command;
    }

    public Command getScoreTwiceCharge2() {
        var thetaController = new ProfiledPIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0, Constants.Auto.KTHETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();
            SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                trajectories.getScoreTwiceCharge2Blue(),
                s_Drivetrain::getPose,
                Constants.DriveTrain.SWERVE_KINEMATICS,
                new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
                new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
                thetaController,
                s_Drivetrain::setModuleStates,
                s_Drivetrain);
    
            command.addCommands(
                new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreTwiceCharge2Blue().getInitialPose())),
                swerveControllerCommand);
            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();
        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectories.getScoreTwiceCharge2Red(),
            s_Drivetrain::getPose,
            Constants.DriveTrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates,
            s_Drivetrain);

        command.addCommands(
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreTwice2Red().getInitialPose())),
            swerveControllerCommand);
        return command;
    }

    public Command getScoreThreeCharge0() {
        var thetaController = new ProfiledPIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0, Constants.Auto.KTHETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();
            SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                trajectories.getScoreThreeCharge0Blue(),
                s_Drivetrain::getPose,
                Constants.DriveTrain.SWERVE_KINEMATICS,
                new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
                new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
                thetaController,
                s_Drivetrain::setModuleStates,
                s_Drivetrain);
    
            command.addCommands(
                new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreThreeCharge0Blue().getInitialPose())),
                swerveControllerCommand);
            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();
        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectories.getScoreThreeCharge0Red(),
            s_Drivetrain::getPose,
            Constants.DriveTrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates,
            s_Drivetrain);

        command.addCommands(
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreThreeCharge0Red().getInitialPose())),
            swerveControllerCommand);
        return command;
    }

    public Command getScoreThreeCharge1() {
        var thetaController = new ProfiledPIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0, Constants.Auto.KTHETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();
            SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                trajectories.getScoreThreeCharge1Blue(),
                s_Drivetrain::getPose,
                Constants.DriveTrain.SWERVE_KINEMATICS,
                new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
                new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
                thetaController,
                s_Drivetrain::setModuleStates,
                s_Drivetrain);
    
            command.addCommands(
                new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreThreeCharge1Blue().getInitialPose())),
                swerveControllerCommand);
            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();
        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectories.getScoreThreeCharge1Red(),
            s_Drivetrain::getPose,
            Constants.DriveTrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates,
            s_Drivetrain);

        command.addCommands(
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreThreeCharge1Red().getInitialPose())),
            swerveControllerCommand);
        return command;
    }
    
    public Command getScoreThreeCharge2() {
        var thetaController = new ProfiledPIDController(Constants.Auto.KP_THETA_CONTROLLER, 0, 0, Constants.Auto.KTHETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        if(DriverStation.getAlliance().name().equals("Blue")){
            SequentialCommandGroup command = new SequentialCommandGroup();
            SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                trajectories.getScoreThreeCharge2Blue(),
                s_Drivetrain::getPose,
                Constants.DriveTrain.SWERVE_KINEMATICS,
                new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
                new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
                thetaController,
                s_Drivetrain::setModuleStates,
                s_Drivetrain);
    
            command.addCommands(
                new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreThreeCharge2Blue().getInitialPose())),
                swerveControllerCommand);
            return command;
        }

        SequentialCommandGroup command = new SequentialCommandGroup();
        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectories.getScoreThreeCharge2Red(),
            s_Drivetrain::getPose,
            Constants.DriveTrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.KP_X_CONTROLLER, 0, 0),
            new PIDController(Constants.Auto.KP_Y_CONTROLLER, 0, 0),
            thetaController,
            s_Drivetrain::setModuleStates,
            s_Drivetrain);

        command.addCommands(
            new InstantCommand(() -> s_Drivetrain.resetOdometry(trajectories.getScoreThreeCharge2Red().getInitialPose())),
            swerveControllerCommand);
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