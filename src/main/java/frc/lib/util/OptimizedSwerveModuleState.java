package frc.lib.util;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

import static frc.lib.math.Boundaries.*;

/** Represents the state of one swerve module. */
@SuppressWarnings("MemberName")
public class OptimizedSwerveModuleState {

  /** Speed of the wheel of the module. */
  public double speedMetersPerSecond;

  /** Angle of the module. */
  public Rotation2d angle = Rotation2d.fromDegrees(0);

  /** Constructs a SwerveModuleState with zeros for speed and angle. */
  public OptimizedSwerveModuleState() {}

  /**
   * Constructs a OptimizedSwerveModuleState.
   *
   * @param SwerveModuleState Wpilib SwerveModuleState
   */
  public OptimizedSwerveModuleState(SwerveModuleState state) {
    this.speedMetersPerSecond = state.speedMetersPerSecond;
    this.angle = state.angle;
  }

  /**
   * Minimize the change in heading the desired swerve module state would require by potentially
   * reversing the direction the wheel spins.
   *
   * @param desiredState The desired state.
   * @param currentAngle The current module angle.
   */
  public SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees();
    if (Math.abs(delta) > 90){
        targetSpeed = -targetSpeed;
        targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
    }        
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }
  
}
