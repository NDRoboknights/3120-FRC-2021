// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.CTRELib.Constant;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
// public class Robot extends TimedRobot {
//   public static CTREConfigs ctreConfigs;

//   private Command m_autonomousCommand;

//   private RobotContainer m_robotContainer;

//   /**
//    * This function is run when the robot is first started up and should be used for any
//    * initialization code.
//    */
//   @Override
//   public void robotInit() {
//     /* Initalizing Configs for CTRE Devices */
//     ctreConfigs = new CTREConfigs();
    
//     // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
//     // autonomous chooser on the dashboard.
//     m_robotContainer = new RobotContainer();
//   }

//   /**
//    * This function is called every robot packet, no matter the mode. Use this for items like
//    * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
//    *
//    * <p>This runs after the mode specific periodic functions, but before LiveWindow and
//    * SmartDashboard integrated updating.
//    */
//   @Override
//   public void robotPeriodic() {
//     // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
//     // commands, running already-scheduled commands, removing finished or interrupted commands,
//     // and running subsystem periodic() methods.  This must be called from the robot's periodic
//     // block in order for anything in the Command-based framework to work.
//     CommandScheduler.getInstance().run();
//   }

//   /** This function is called once each time the robot enters Disabled mode. */
//   @Override
//   public void disabledInit() {}

//   @Override
//   public void disabledPeriodic() {}

//   /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
//   @Override
//   public void autonomousInit() {
//     m_autonomousCommand = m_robotContainer.getAutonomousCommand();

//     // schedule the autonomous command (example)
//     if (m_autonomousCommand != null) {
//       m_autonomousCommand.schedule();
//     }
//   }

//   /** This function is called periodically during autonomous. */
//   @Override
//   public void autonomousPeriodic() {}

//   @Override
//   public void teleopInit() {
//     // This makes sure that the autonomous stops running when
//     // teleop starts running. If you want the autonomous to
//     // continue until interrupted by another command, remove
//     // this line or comment it out.
//     if (m_autonomousCommand != null) {
//       m_autonomousCommand.cancel();
//     }
//   }

//   /** This function is called periodically during operator control. */
//   @Override
//   public void teleopPeriodic() {}

//   @Override
//   public void testInit() {
//     // Cancels all running commands at the start of test mode.
//     CommandScheduler.getInstance().cancelAll();
//   }

//   /** This function is called periodically during test mode. */
//   @Override
//   public void testPeriodic() {}
//}
public class Robot extends TimedRobot {

public static CTREConfigs ctreConfigs = new CTREConfigs();

/** Hardware */
TalonFX _talon = new TalonFX(2);
XboxController _joy = new XboxController(0);

  /** Used to create string thoughout loop */
StringBuilder _sb = new StringBuilder();
int _loops = 0;

  /** Track button state for single press event */
boolean _lastButton1 = false;

/** Save the target position to servo to */
double targetPositionRotations;

public void robotInit() {
  /* Factory Default all hardware to prevent unexpected behaviour */
  _talon.configFactoryDefault();
  
  /* Config the sensor used for Primary PID and sensor direction */
      _talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 
                                          Constant.kPIDLoopIdx,
                                  Constant.kTimeoutMs);

  /* Ensure sensor is positive when output is positive */
  _talon.setSensorPhase(Constant.kSensorPhase);

  /**
   * Set based on what direction you want forward/positive to be.
   * This does not affect sensor phase. 
   */ 
  _talon.setInverted(Constant.kMotorInvert);
  /*
   * Talon FX does not need sensor phase set for its integrated sensor
   * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
   * and the user calls getSelectedSensor* to get the sensor's position/velocity.
   * 
   * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
   */
      // _talon.setSensorPhase(true);

  /* Config the peak and nominal outputs, 12V means full */
  _talon.configNominalOutputForward(0, Constant.kTimeoutMs);
  _talon.configNominalOutputReverse(0, Constant.kTimeoutMs);
  _talon.configPeakOutputForward(1, Constant.kTimeoutMs);
  _talon.configPeakOutputReverse(-1, Constant.kTimeoutMs);

  /**
   * Config the allowable closed-loop error, Closed-Loop output will be
   * neutral within this range. See Table in Section 17.2.1 for native
   * units per rotation.
   */
  _talon.configAllowableClosedloopError(0, Constant.kPIDLoopIdx, Constant.kTimeoutMs);

  /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
  _talon.config_kF(Constant.kPIDLoopIdx, Constant.kGains.kF, Constant.kTimeoutMs);
  _talon.config_kP(Constant.kPIDLoopIdx, Constant.kGains.kP, Constant.kTimeoutMs);
  _talon.config_kI(Constant.kPIDLoopIdx, Constant.kGains.kI, Constant.kTimeoutMs);
  _talon.config_kD(Constant.kPIDLoopIdx, Constant.kGains.kD, Constant.kTimeoutMs);
  }
  
void commonLoop() {
  /* Gamepad processing */
  double leftYstick = _joy.getY(Hand.kLeft);
  boolean button1 = _joy.getXButton();	// X-But;ton
  boolean button2 = _joy.getAButton();	// A-Button

  /* Get Talon's current output percentage */
  double motorOutput = _talon.getMotorOutputPercent();

  /* Deadband gamepad */
  if (Math.abs(leftYstick) < 0.10) {
    /* Within 10% of zero */
    leftYstick = 0;
  }

  /* Prepare line to print */
  _sb.append("\tout:");
  /* Cast to int to remove decimal places */
  _sb.append((int) (motorOutput * 100));
  _sb.append("%");	// Percent

  _sb.append("\tpos:");
  _sb.append(_talon.getSelectedSensorPosition(0));
  _sb.append("u"); 	// Native units

  /**
   * When button 1 is pressed, perform Position Closed Loop to selected position,
   * indicated by Joystick position x10, [-10, 10] rotations
   */
  if (!_lastButton1 && button1) {
    /* Position Closed Loop */

    /* 10 Rotations * 2048 u/rev in either direction */
    targetPositionRotations = leftYstick * 10.0 * 2048;
    _talon.set(TalonFXControlMode.Position, targetPositionRotations);
  }

  /* When button 2 is held, just straight drive */
  if (button2) {
    /* Percent Output */

    _talon.set(TalonFXControlMode.PercentOutput, leftYstick);
  }

  /* If Talon is in position closed-loop, print some more info */
  if (_talon.getControlMode() == TalonFXControlMode.Position.toControlMode()) {
    /* ppend more signals to print when in speed mode. */
    _sb.append("\terr:");
    _sb.append(_talon.getClosedLoopError(0));
    _sb.append("u");	// Native Units

    _sb.append("\ttrg:");
    _sb.append(targetPositionRotations);
    _sb.append("u");	/// Native Units
  }

  /**
   * Print every ten loops, printing too much too fast is generally bad
   * for performance.
   */
  if (++_loops >= 10) {
    _loops = 0;
    System.out.println(_sb.toString());
  }

  /* Reset built string for next loop */
  _sb.setLength(0);
  
  /* Save button state for on press detect */
  _lastButton1 = button1;
  }
  
/**
 * This function is called periodically during operator control
 */
public void teleopPeriodic() {
  commonLoop();
}
}