package frc.robot.subsystems;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.robot.Constants;

import frc.lib.util.OptimizedSwerveModuleState;
import static frc.lib.math.Conversions.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public class SwerveModule {
    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private TalonFXConfiguration angleMotorConfig;
    private TalonFXConfiguration driveMotorConfig;
    private double offset;
    private boolean invertDrive;
    private boolean invertAngle;
    private double lastAngle;

    public SwerveModule(double offset, int angleMotorID, int driveMotorID, boolean invertDrive, boolean invertAngle, TalonFXConfiguration angleMotorConfig, TalonFXConfiguration driveMotorConfig){
        this.offset = offset;
        this.angleMotorConfig = angleMotorConfig;
        this.driveMotorConfig = driveMotorConfig;
        this.invertDrive = invertDrive;
        this.invertAngle = invertAngle;
        
        /* Absolute Encoder Config */
        //TODO: Do any config you need for the encoder

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(angleMotorID);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(driveMotorID);
        configDriveMotor();

        lastAngle = getAngle().getDegrees();
    }

    public void setDesiredState(SwerveModuleState desiredState){
        SwerveModuleState state = new OptimizedSwerveModuleState(desiredState).optimize(desiredState, getAngle()); //Custom SwerveModuleState used only here for the customized .optimize function to work with CTRE

        mDriveMotor.set(ControlMode.Velocity, toFalconFromMetersPerSecond(state.speedMetersPerSecond, Constants.Swerve.wheelDiameter, Constants.Swerve.driveGearRatio));

        double angle = (Math.abs(state.speedMetersPerSecond) <= (Constants.Swerve.maxDriveSpeed* 0.05)) ? lastAngle : state.angle.getDegrees(); //If the specified drive output is very small and not enough to actually move the robot, then don't move the module angle (AKA a Deadzone) 
        mAngleMotor.set(ControlMode.MotionMagic, toFalconFromDegrees(angle, Constants.Swerve.angleGearRatio));
        lastAngle = angle;
    }

    /* Sets Falcon to absolute position from Absolute Encoder */
    private void resetToAbsolute(){
        int absolutePosition = toFalconFromDegrees(getAbsolutePos() + offset, Constants.Swerve.angleGearRatio);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    /* Configure the Angle Falcon */
    private void configAngleMotor(){
        mAngleMotor.configFactoryDefault();
        mAngleMotor.configAllSettings(angleMotorConfig, Constants.ctreTimeout);
        mAngleMotor.setInverted(invertAngle);
        mAngleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
        mAngleMotor.setStatusFramePeriod(StatusFrame.Status_1_General, Constants.Swerve.angleStatus1, Constants.ctreTimeout);
        mAngleMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, Constants.Swerve.angleStatus2, Constants.ctreTimeout);
        mAngleMotor.configNeutralDeadband(0.04, Constants.ctreTimeout); //0.001 for tuning
        resetToAbsolute();
    }

    /* Configure the Drive Falcon */
    private void configDriveMotor(){        
        mDriveMotor.configFactoryDefault();
        mDriveMotor.configAllSettings(driveMotorConfig);
        mDriveMotor.setInverted(invertDrive);
        mDriveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
        mDriveMotor.setStatusFramePeriod(StatusFrame.Status_1_General, Constants.Swerve.driveStatus1);
        mDriveMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, Constants.Swerve.driveStatus2);
    }

    public double getAbsolutePos(){
        return 0; //TODO: Replace with however the encoder you use works
    }

    public Rotation2d getAngle(){
        return Rotation2d.fromDegrees(toDegreesFromFalcon(mAngleMotor.getSelectedSensorPosition(), Constants.Swerve.angleGearRatio));
    }

    public double getVelocity(){
        double wheelRPM = toRPMFromFalcon(mDriveMotor.getSelectedSensorVelocity(), Constants.Swerve.driveGearRatio);
        double wheelMPS = (wheelRPM * Constants.Swerve.wheelCircumference) / 60;
        return wheelMPS;
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getVelocity(), getAngle());
    }
    
}