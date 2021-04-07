// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSystem extends SubsystemBase 
{
  public CANSparkMax s1, s2;

  private CANPIDController s1PIDController, s2PIDController;
  
  /** Creates a new ShooterSystem. */
  public ShooterSystem() 
  {
      s1 = new CANSparkMax(Constants.RestOfRobot.shooterNEOOne, MotorType.kBrushless);
      s2 = new CANSparkMax(Constants.RestOfRobot.shooterNEOTwo, MotorType.kBrushless);

      /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    s1.restoreFactoryDefaults();

    // initialze PID controller and encoder objects
    s1PIDController = s1.getPIDController();
    s2PIDController = s2.getPIDController();

    s1PIDController.setP(Constants.RestOfRobot.Shooter.kP);
    s1PIDController.setI(Constants.RestOfRobot.Shooter.kI);
    s1PIDController.setD(Constants.RestOfRobot.Shooter.kD);
    s1PIDController.setIZone(Constants.RestOfRobot.Shooter.kIz);
    s1PIDController.setFF(Constants.RestOfRobot.Shooter.kFF);
    s1PIDController.setOutputRange(Constants.RestOfRobot.Shooter.kMinOutput, Constants.RestOfRobot.Shooter.kMaxOutput);

    s2PIDController.setP(Constants.RestOfRobot.Shooter.kP);
    s2PIDController.setI(Constants.RestOfRobot.Shooter.kI);
    s2PIDController.setD(Constants.RestOfRobot.Shooter.kD);
    s2PIDController.setIZone(Constants.RestOfRobot.Shooter.kIz);
    s2PIDController.setFF(Constants.RestOfRobot.Shooter.kFF);
    s2PIDController.setOutputRange(Constants.RestOfRobot.Shooter.kMinOutput, Constants.RestOfRobot.Shooter.kMaxOutput);

  }

  public void eStop()
  {
    s1.set(0);
    s2.set(0);
  }

  @Override
  public void periodic() 
  {
    SmartDashboard.putNumber("Shooter velocity: ", (s1.getEncoder().getVelocity() + s2.getEncoder().getVelocity())/2.0);
  }

  public void setVelocity(double revolutionsPerMinute)
  {
    s1PIDController.setReference(revolutionsPerMinute, ControlType.kVelocity);
    s2PIDController.setReference(revolutionsPerMinute, ControlType.kVelocity);
  }

}
