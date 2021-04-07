// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexSystem extends SubsystemBase 
{

  public CANSparkMax indexerNEOMotor;
  public TalonSRX vHopperOne, vHopperTwo, vHopperThree;
  /** Creates a new IndexSystem. */
  public IndexSystem() 
  {
      this.indexerNEOMotor = new CANSparkMax(Constants.RestOfRobot.indexerNEOID, MotorType.kBrushless);

      this.vHopperOne = new TalonSRX(Constants.RestOfRobot.vHopper775One);
      this.vHopperTwo = new TalonSRX(Constants.RestOfRobot.vHopper775Two);
      this.vHopperThree = new TalonSRX(Constants.RestOfRobot.vHopper775Three);

  }

  @Override
  public void periodic() 
  {
      if(!(vHopperOne.getStatorCurrent() > 0.25 || vHopperTwo.getStatorCurrent() > 0.25 || vHopperThree.getStatorCurrent() > 0.25)){
          SmartDashboard.putString("Hopper state: ", State.IDLE.toString());
      }else{
          SmartDashboard.putString("Hopper state:", State.RUNNING.toString());
      }

      if(!(indexerNEOMotor.getOutputCurrent() > 1.5)){
          SmartDashboard.putString("Indexer state: ", State.IDLE.toString());
      }else if(indexerNEOMotor.getEncoder(EncoderType.kHallSensor, 42).getVelocity() < 4500){
          SmartDashboard.putString("Indexer state: ", State.RUNNING.toString());
      }else{
          SmartDashboard.putString("Indexer state: ", State.SPIT.toString());
      }
  }

  public void eStop()
  {
      indexerNEOMotor.set(0);
      vHopperOne.set(TalonSRXControlMode.PercentOutput, 0);
      vHopperTwo.set(TalonSRXControlMode.PercentOutput, 0);
      vHopperThree.set(TalonSRXControlMode.PercentOutput, 0);
  }

  public void setRawPower(double pow)
  {
    indexerNEOMotor.set(pow);
    vHopperOne.set(TalonSRXControlMode.PercentOutput, pow);
    vHopperTwo.set(TalonSRXControlMode.PercentOutput, pow);
    vHopperThree.set(TalonSRXControlMode.PercentOutput, pow);
  }

  public static enum Directions
  {
    FORWARDS(1), BACKWARDS(-1);
    
    private int v;
    Directions(int v){
      this.v = v;
    }

    public int getInt(){ return this.v; }
    
  }

  private enum State
  {
      IDLE("IDLE"), IN("IN"), OUT("OUT"), SPIT("SPIT"), RUNNING("RUNNING");
      private String s;
      
      State(String s){
        this.s = s;
      }

      public String toString(){ return this.s; }
  }
}
