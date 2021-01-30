// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class SwerveDriveSystem extends SubsystemBase {
  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule m_frontLeft;
  private SwerveModule m_frontRight;
  private SwerveModule m_rearLeft;
  private SwerveModule m_rearRight;

  public SwerveDriveSystem(){
    zeroGyro();
    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.sdKinematics, getGyroHeading());

    m_frontLeft =
      new SwerveModule( 
        Constants.Swerve.Mod1.angleOffset, Constants.Swerve.Mod1.angleID, Constants.Swerve.Mod1.driveID,
        Constants.Swerve.Mod1.driveInvert, Constants.Swerve.Mod1.angleInvert,
        Robot.ctreConfigs.mod1AngleFXConfig, Robot.ctreConfigs.swerveDriveFXConfig);

    m_frontRight =
      new SwerveModule( 
        Constants.Swerve.Mod2.angleOffset, Constants.Swerve.Mod2.angleID, Constants.Swerve.Mod2.driveID,
        Constants.Swerve.Mod2.driveInvert, Constants.Swerve.Mod2.angleInvert,
        Robot.ctreConfigs.mod2AngleFXConfig, Robot.ctreConfigs.swerveDriveFXConfig);
          
    m_rearLeft =
      new SwerveModule( 
        Constants.Swerve.Mod3.angleOffset, Constants.Swerve.Mod3.angleID, Constants.Swerve.Mod3.driveID,
        Constants.Swerve.Mod3.driveInvert, Constants.Swerve.Mod3.angleInvert,
        Robot.ctreConfigs.mod3AngleFXConfig, Robot.ctreConfigs.swerveDriveFXConfig);
        
    m_rearRight =
      new SwerveModule( 
        Constants.Swerve.Mod4.angleOffset, Constants.Swerve.Mod4.angleID, Constants.Swerve.Mod4.driveID,
        Constants.Swerve.Mod4.driveInvert, Constants.Swerve.Mod4.angleInvert,
        Robot.ctreConfigs.mod4AngleFXConfig, Robot.ctreConfigs.swerveDriveFXConfig);
  }

  /* Used for standard Drive */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.sdKinematics.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                translation.getX(), 
                                translation.getY(), 
                                rotation, 
                                getGyroHeading()
                            )
                            : new ChassisSpeeds(
                                translation.getX(), 
                                translation.getY(), 
                                rotation)
                            );
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, Constants.Swerve.maxDriveSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);

  }

  /* Used by swerve controller in auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
      SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, Constants.Swerve.maxDriveSpeed);
      m_frontLeft.setDesiredState(desiredStates[0]);
      m_frontRight.setDesiredState(desiredStates[1]);
      m_rearLeft.setDesiredState(desiredStates[2]);
      m_rearRight.setDesiredState(desiredStates[3]);
  }

  public Pose2d getSwervePose() {
      return swerveOdometry.getPoseMeters();
  }

  public void resetSwerveOdometry(Pose2d pose) {
      swerveOdometry.resetPosition(pose, getGyroHeading());
  }

  public SwerveModuleState[] getStates(){
      SwerveModuleState[] states = new SwerveModuleState[4];
      states[0] = m_frontLeft.getState();
      states[1] = m_frontRight.getState();
      states[2] = m_rearLeft.getState();
      states[3] = m_rearRight.getState();
      return states;
  }

  public void zeroGyro(){
      //TODO: Zero Your Gyro
  }

  public Rotation2d getGyroHeading() {
      //TODO: Setup Your Gyro
      return new Rotation2d();
  }  

  @Override
  public void periodic(){    
    swerveOdometry.update(getGyroHeading(), getStates());
  }
}