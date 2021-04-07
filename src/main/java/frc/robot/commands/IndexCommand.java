// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexSystem;

public class IndexCommand extends CommandBase 
{
  IndexSystem indexSys;
  XboxController controller;
  /** Creates a new IndexCommand. */
  public IndexCommand(IndexSystem c_Index, XboxController c_controller) 
  {
    this.indexSys = c_Index;
    this.controller = c_controller;

    addRequirements(indexSys);
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
      double power = controller.getTriggerAxis(Hand.kRight);

      if(Math.abs(power) > 0.05)
      {
        indexSys.setRawPower(power);
      }else{
        indexSys.setRawPower(0);
      }

      if(controller.getBumper(Hand.kRight))
      {
        indexSys.setRawPower(-1);
      }else{
        indexSys.setRawPower(0);
      }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    indexSys.eStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
