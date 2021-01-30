package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

public final class CTREConfigs {

    public TalonFXConfiguration mod1AngleFXConfig;
    public TalonFXConfiguration mod2AngleFXConfig;
    public TalonFXConfiguration mod3AngleFXConfig;
    public TalonFXConfiguration mod4AngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;

    public CTREConfigs(){

        mod1AngleFXConfig = new TalonFXConfiguration();
        mod2AngleFXConfig = new TalonFXConfiguration();
        mod3AngleFXConfig = new TalonFXConfiguration();
        mod4AngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();

        
        /* Swerve Angle Motor Configurations */

        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.Swerve.angleEnableCurrentLimit, 
            Constants.Swerve.angleContinuousCurrentLimit, 
            Constants.Swerve.anglePeakCurrentLimit, 
            Constants.Swerve.anglePeakCurrentDuration);

        //Mod 0
        mod1AngleFXConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        mod1AngleFXConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        mod1AngleFXConfig.slot0.kP = Constants.Swerve.angleKP;
        mod1AngleFXConfig.slot0.kI = Constants.Swerve.angleKI;
        mod1AngleFXConfig.slot0.kD = Constants.Swerve.angleKD;
        mod1AngleFXConfig.slot0.kF = Constants.Swerve.angleKF;
        mod1AngleFXConfig.supplyCurrLimit = angleSupplyLimit;
        mod1AngleFXConfig.motorCommutation = MotorCommutation.Trapezoidal;
        mod1AngleFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

        //Mod 1
        mod2AngleFXConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        mod2AngleFXConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        mod2AngleFXConfig.slot0.kP = Constants.Swerve.angleKP;
        mod2AngleFXConfig.slot0.kI = Constants.Swerve.angleKI;
        mod2AngleFXConfig.slot0.kD = Constants.Swerve.angleKD;
        mod2AngleFXConfig.slot0.kF = Constants.Swerve.angleKF;  
        mod2AngleFXConfig.supplyCurrLimit = angleSupplyLimit;
        mod2AngleFXConfig.motorCommutation = MotorCommutation.Trapezoidal;
        mod2AngleFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

        //Mod 2
        mod3AngleFXConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        mod3AngleFXConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        mod3AngleFXConfig.slot0.kP = Constants.Swerve.angleKP;
        mod3AngleFXConfig.slot0.kI = Constants.Swerve.angleKI;
        mod3AngleFXConfig.slot0.kD = Constants.Swerve.angleKD;
        mod3AngleFXConfig.slot0.kF = Constants.Swerve.angleKF;      
        mod3AngleFXConfig.supplyCurrLimit = angleSupplyLimit;
        mod3AngleFXConfig.motorCommutation = MotorCommutation.Trapezoidal;
        mod3AngleFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

        //Mod 3
        mod4AngleFXConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        mod4AngleFXConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        mod4AngleFXConfig.slot0.kP = Constants.Swerve.angleKP;
        mod4AngleFXConfig.slot0.kI = Constants.Swerve.angleKI;
        mod4AngleFXConfig.slot0.kD = Constants.Swerve.angleKD;
        mod4AngleFXConfig.slot0.kF = Constants.Swerve.angleKF;
        mod4AngleFXConfig.supplyCurrLimit = angleSupplyLimit;
        mod4AngleFXConfig.motorCommutation = MotorCommutation.Trapezoidal;
        mod1AngleFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;


        /* Swerve Drive Motor Configuration */

        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.Swerve.driveEnableCurrentLimit, 
            Constants.Swerve.driveContinuousCurrentLimit, 
            Constants.Swerve.drivePeakCurrentLimit, 
            Constants.Swerve.drivePeakCurrentDuration);
        
        //All Mods
        swerveDriveFXConfig.slot0.kP = Constants.Swerve.driveKP;
        swerveDriveFXConfig.slot0.kI = Constants.Swerve.driveKI;
        swerveDriveFXConfig.slot0.kD = Constants.Swerve.driveKD;
        swerveDriveFXConfig.slot0.kF = Constants.Swerve.driveKF;        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.motorCommutation = MotorCommutation.Trapezoidal;
        swerveDriveFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        swerveDriveFXConfig.openloopRamp = Constants.Swerve.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = Constants.Swerve.closedLoopRamp;
    }
}