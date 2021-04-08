package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;



public class Vision extends SubsystemBase {
    private MedianFilter distanceMedian = new MedianFilter(10);

    public Vision() {}

    /**@return Horizontal Offset From Crosshair To Target in degrees
     * (Note: Inverted from standard LL provided angle to be CCW+) */
    public Rotation2d getTx() {
        return Rotation2d.fromDegrees(-NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0));
    }

    /**@return Vertical Offset from Crosshair to Target in degrees */
    public Rotation2d getTy() {
        return Rotation2d.fromDegrees(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0));
    }

    /**@return True if LL detects Target */
    public boolean hasTarget(){
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1 ? true : false;
    }

    /**@return Median filtered distance to Target in meters */
    public Translation2d getDistance(){
        double heightDifference = Constants.Vision.goalHeight - Constants.Vision.limelightHeight;
        Rotation2d combinedAngle = Constants.Vision.limelightAngle.plus(getTy());
        return new Translation2d(distanceMedian.calculate((heightDifference / combinedAngle.getTan())), 0);
    }

    /**@return The pipeline’s latency contribution in seconds  */
    public double getLatency() {
        double ntLatency = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl").getDouble(0);
        return ((ntLatency + 11) / 1000); //11ms Capture Latency
    }

    /**Use to set state of LL's leds
     * @param ledState default, off, blink, on
     */
    public void ledState(String ledState){
        switch (ledState){
            case "default":
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
                break;
            case "off":
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
                break;
            case "blink":
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2);
                break;
            case "on":
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
                break;
        }
    }

    /** Use to set LL camera mode
     * @param camMode driver, vision
     */
    public void camMode(String camMode){
        switch (camMode){
            case "driver":
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
                break;
            case "vision":
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
                break;
        }
    }

    /** Use to set active pipeline
     * @param pipeline 0-9
     */
    public void setPipeline(int pipeline){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
    }
    
    @Override
    public void periodic(){
        if(!hasTarget()){
            distanceMedian.reset();
        }
    }
}