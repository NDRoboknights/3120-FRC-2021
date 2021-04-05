// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;


public final class Constants {

	public static final int ctreTimeout = 10;

	public static final class Swerve{

		/* Swerve Velocity Values */
		public static final double maxDriveSpeed = Units.feetToMeters(16.2); // TODO: replace the 16.2 with a tested maximum value.
		public static final double maxAngularSpeed = 11.5; //This is okay being theoretical, dont need to test

		/* Drivetrain Constants */
		public static final double wheelBase = Units.inchesToMeters(25.5);
		public static final double trackWidth = Units.inchesToMeters(21.5);
		public static final double wheelDiameter = Units.inchesToMeters(4);
		public static final double wheelCircumference = wheelDiameter * Math.PI;

		public static final double driveGearRatio = (6.84);
		public static final double angleGearRatio = (12.8);

		/* Distance between front and back wheels on robot */
		public static final SwerveDriveKinematics sdKinematics = new SwerveDriveKinematics(
				new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
				new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
				new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
				new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

		/* Swerve Current Limiting */
		public static final int angleContinuousCurrentLimit = 25;
		public static final int anglePeakCurrentLimit = 30;
		public static final double anglePeakCurrentDuration = 0.1;
		public static final boolean angleEnableCurrentLimit = true;

		public static final int driveContinuousCurrentLimit = 35;
		public static final int drivePeakCurrentLimit = 60;
		public static final double drivePeakCurrentDuration = 0.1;
		public static final boolean driveEnableCurrentLimit = true;

		/* Angle Motor PID Values */
		public static final double angleKP = 0.6;
		public static final double angleKI = 0.0;
		public static final double angleKD = 10.0;
		public static final double angleKF = 0.0;

		/* Drive Motor PID Values */
		public static final double driveKP = 0.0;
		public static final double driveKI = 0.0;
		public static final double driveKD = 0.0;
		public static final double driveKF = 0.0;

		/* Drive Ramping */
		public static final double openLoopRamp = 0.25;
		public static final double closedLoopRamp = 0.0;

		/* Neutral Modes */
		public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
		public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

		/* Status Frame Update Periods */
		public static final int angleStatus1 = 10;
		public static final int angleStatus2 = 20;
		public static final int driveStatus1 = 10;
		public static final int driveStatus2 = 20;

		/* Module Specific Constants */
		/** Front Left Module */
		public static final class Mod1 {
			public static final int encID = 0;
			public static final int driveID = 1;
			public static final int angleID = 0;
			public static final boolean driveInvert = false;
			public static final boolean angleInvert = false;
			public static final double angleOffset = -149.57;
		}

		/** Front Right Module */
		public static final class Mod2 {
			public static final int encID = 1;
			public static final int driveID = 2;
			public static final int angleID = 6;
			public static final boolean driveInvert = false;
			public static final boolean angleInvert = false;
			public static final double angleOffset = -12.19;
		}
		
		/** Back Left Module */
		public static final class Mod3 {
			public static final int encID = 2;
			public static final int driveID = 3;
			public static final int angleID = 7;
			public static final boolean driveInvert = false;
			public static final boolean angleInvert = false;
			public static final double angleOffset = -7.03;
		}

		/** Back Right Module */
		public static final class Mod4 {
			public static final int encID = 3;
			public static final int driveID = 5;
			public static final int angleID = 4;
			public static final boolean driveInvert = false;
			public static final boolean angleInvert = false;
			public static final double angleOffset = -311.59;
		}

	}
	
		//Auton Constants
	public static final class AutoConstants
	{
		public static final double kMaxSpeedMetersPerSecond = Swerve.maxDriveSpeed - 0.5;
		public static final double kMaxAccelerationMetersPerSecondSquared = 4;

		public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
		
		public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
		new TrapezoidProfile.Constraints(
			kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
	}
	
}
