package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSystem;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class DriveCommand extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private Boolean fieldRelative = true;
    
    private SwerveDriveSystem s_Swerve;
    private XboxController controller;

    /**
     * Driver control
     */
    public DriveCommand(SwerveDriveSystem s_Swerve, XboxController controller) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.controller = controller;
    }

    @Override
    public void execute() {
        double yAxis = controller.getY(GenericHID.Hand.kLeft);
        double xAxis = controller.getX(GenericHID.Hand.kLeft);
        double rAxis = controller.getX(GenericHID.Hand.kRight);

        /* Deadbands, probably a better way of doing this */
        if (Math.abs(yAxis) < 0.1){
            yAxis = 0;
        }
        if (Math.abs(xAxis) < 0.1){
            xAxis = 0;
        }
        if (Math.abs(rAxis) < 0.1){
            rAxis = 0;
        }

        translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxDriveSpeed);
        rotation = rAxis * Constants.Swerve.maxAngularSpeed;
        s_Swerve.drive(translation, rotation, fieldRelative);

        SmartDashboard.putNumber("Swerve Module 1 Enc: ", s_Swerve.getStates()[0].angle.getDegrees());
        SmartDashboard.putNumber("Swerve Module 2 Enc: ", s_Swerve.getStates()[1].angle.getDegrees());
        SmartDashboard.putNumber("Swerve Module 3 Enc: ", s_Swerve.getStates()[2].angle.getDegrees());
        SmartDashboard.putNumber("Swerve Module 4 Enc: ", s_Swerve.getStates()[3].angle.getDegrees());
    }
}
