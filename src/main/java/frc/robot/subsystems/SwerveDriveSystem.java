// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
 
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class SwerveDriveSystem extends SubsystemBase 
{
  /** Creates a new SwerveDriveSystem. */
  private TalonFX steerFrontRight, steerFrontLeft, steerBackRight, steerBackLeft,
    driveFrontRight, driveFrontLeft, driveBackRight, driveBackLeft;
  
  private static final Translation2d moduleFrontLeft = new Translation2d(Units.inchesToMeters(12.75), Units.inchesToMeters(10.75));
  private static final Translation2d moduleFrontRight = new Translation2d(Units.inchesToMeters(12.75), Units.inchesToMeters(-10.75));
  private static final Translation2d moduleBackLeft = new Translation2d(Units.inchesToMeters(-12.75), Units.inchesToMeters(10.75));
  private static final Translation2d moduleBackRight = new Translation2d(Units.inchesToMeters(-12.75), Units.inchesToMeters(-10.75));

  private SwerveDriveKinematics sdKinematics = new SwerveDriveKinematics(moduleFrontLeft, moduleFrontRight, moduleBackLeft, moduleBackRight);

  public SwerveDriveSystem(int... ids)
  {
    steerFrontRight = new TalonFX(ids[0]);
    driveFrontRight = new TalonFX(ids[1]);

    steerFrontLeft = new TalonFX(ids[2]);
    driveFrontLeft = new TalonFX(ids[3]);

    steerBackRight = new TalonFX(ids[4]);
    driveBackRight = new TalonFX(ids[5]);
    
    steerBackLeft = new TalonFX(ids[6]);
    driveBackLeft = new TalonFX(ids[7]);
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
    ChassisSpeeds speeds = controllerToSpeeds();

    SwerveModuleState[] states = sdKinematics.toSwerveModuleStates(speeds);

    driveFrontLeft.set(ControlMode.Velocity, states[0].speedMetersPerSecond);
    steerFrontLeft.set(ControlMode.Position, states[0].angle.getDegrees());

    driveFrontRight.set(ControlMode.Velocity, states[1].speedMetersPerSecond);
    steerFrontRight.set(ControlMode.Position, states[1].angle.getDegrees());

    driveBackLeft.set(ControlMode.Velocity, states[2].speedMetersPerSecond);
    steerBackLeft.set(ControlMode.Position, states[2].angle.getDegrees());

    driveBackRight.set(ControlMode.Velocity, states[3].speedMetersPerSecond);
    steerBackRight.set(ControlMode.Position, states[3].angle.getDegrees());
  }

  private ChassisSpeeds controllerToSpeeds()
  {
    double x = Units.feetToMeters(RobotContainer.controller.getX(Hand.kLeft) * Constants.DRIVE_MAX_SPEED);
    double y = Units.feetToMeters(RobotContainer.controller.getY(Hand.kLeft) * Constants.DRIVE_MAX_SPEED);

    double theta = Math.atan2(y, x);

    return new ChassisSpeeds(x, y, theta);
  }

  public void simulationPeriodic(){}
}