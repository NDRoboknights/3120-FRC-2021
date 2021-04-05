package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSystem;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class Bounce extends SequentialCommandGroup
 {
    public Bounce(SwerveDriveSystem s_Swerve){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.sdKinematics);

        Trajectory bounce =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.feetToMeters(2.5), Units.feetToMeters(7.5),  Rotation2d.fromDegrees(0)),
                List.of(
                   new Translation2d(Units.feetToMeters(7.5), Units.feetToMeters(11.5)),
                   new Translation2d(Units.feetToMeters(12.5), Units.feetToMeters(2)),
                   new Translation2d(Units.feetToMeters(15), Units.feetToMeters(5)),
                   new Translation2d(Units.feetToMeters(15), Units.feetToMeters(11.5)),
                   new Translation2d(Units.feetToMeters(16), Units.feetToMeters(2)),
                   new Translation2d(Units.feetToMeters(22.5), Units.feetToMeters(5)),
                   new Translation2d(Units.feetToMeters(22.5), Units.feetToMeters(11.5)),
                   new Translation2d(Units.feetToMeters(22.5), Units.feetToMeters(7.5))
                ),
                new Pose2d(Units.feetToMeters(27.5), Units.feetToMeters(7.5),  Rotation2d.fromDegrees(0)),
                config);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                bounce,
                s_Swerve::getSwervePose,
                Constants.Swerve.sdKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);


        addCommands(
            new InstantCommand(() -> s_Swerve.resetSwerveOdometry(bounce.getInitialPose())),
            swerveControllerCommand
        );
    }

	public Bounce(Command... commands) {
		super(commands);
	}
}