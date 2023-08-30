package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private Drivetrain s_Drivetrain;
    private Joystick translationalController;
    private Joystick rotationalController;
    private XboxController xboxController;
    private int translationAxis;
    private int strafeAxis;
    private int rotationAxis;
    boolean foc;


    /**
     * Driver control
     */
    public TeleopSwerve(Drivetrain s_Drivetrain, Joystick controller, Joystick controller1, XboxController xboxController, int translationAxis, int strafeAxis, int rotationAxis, boolean fieldRelative, boolean openLoop) {
        this.s_Drivetrain = s_Drivetrain;
        addRequirements(s_Drivetrain);

        this.translationalController = controller;
        this.rotationalController = controller1;
        this.xboxController = xboxController;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.rotationAxis = rotationAxis;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
    }

    @Override
    public void execute() {
        double yAxis = translationalController.getRawAxis(strafeAxis);
        double xAxis = translationalController.getRawAxis(translationAxis);
        double rAxis = rotationalController.getRawAxis(rotationAxis);
        
        /* Deadbands */
        yAxis = (Math.abs(yAxis) < Constants.Controllers.STICK_DEADBAND) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.Controllers.STICK_DEADBAND) ? 0 : xAxis;
        rAxis = (Math.abs(rAxis) < Constants.Controllers.STICK_DEADBAND) ? 0 : rAxis;

        translation = new Translation2d(yAxis, xAxis).times(Constants.Drivetrain.MAX_SPEED);
        rotation = rAxis * Constants.Drivetrain.MAX_ANGULAR_VELOCITY;
        foc = xboxController.getRawButton(9);
        s_Drivetrain.drive(translation, rotation, fieldRelative, openLoop, false);
    }
}
