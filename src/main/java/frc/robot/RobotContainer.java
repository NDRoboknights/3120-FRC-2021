// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.RestOfRobot.Shooter;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.IndexSystem;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.subsystems.SwerveDriveSystem;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	/* Configure Controllers */
	private final XboxController m_driverController = new XboxController(0);

	/* Configure Buttons */
	private final JoystickButton zeroGyro = new JoystickButton(m_driverController, 4);

	/* Configure Subsystems */
	private final SwerveDriveSystem s_Swerve = new SwerveDriveSystem();
	private final Vision s_Vision = new Vision();
	private final ShooterSystem s_Shooter = new ShooterSystem();
	private final IndexSystem s_Index = new IndexSystem();


	
	public RobotContainer() {
		s_Swerve.setDefaultCommand(new DriveCommand(s_Swerve, m_driverController));
		s_Shooter.setDefaultCommand(new ShooterCommand(s_Shooter, m_driverController));
		s_Index.setDefaultCommand(new IndexCommand(s_Index, m_driverController));

		configureButtonBindings();
	}

	private void configureButtonBindings() {
		zeroGyro.whenPressed(new InstantCommand(() -> zeroGyro()));

	}

	public void zeroGyro(){
		s_Swerve.zeroGyro();
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return null;
	}
}
