// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSystem;

public class ShooterCommand extends CommandBase 
{
  private ShooterSystem sSystem;
  private XboxController controller;

  /** Creates a new ShooterCommand. */
  public ShooterCommand(ShooterSystem sSystem, XboxController controller)
  {
    addRequirements(sSystem);
    this.sSystem = sSystem;
    this.controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
      if(Math.abs(controller.getTriggerAxis(Hand.kLeft)) > 0.05)
      {
        sSystem.setVelocity(Constants.RestOfRobot.Shooter.shotRPM);
      }else{
        sSystem.setVelocity(0);
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
      sSystem.eStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
