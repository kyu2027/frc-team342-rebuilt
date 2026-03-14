// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.CustomXboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {
  private SparkFlex turretMotor;
  private SparkFlexConfig turretConfig;
  private RelativeEncoder turretEncoder;
  private SparkClosedLoopController turretController;

  private SwerveDrive swerve;

  private double goal;
  private boolean manual;

  /** Creates a new Turret. */
  public Turret(SwerveDrive swerve) {
    this.swerve = swerve;

    turretMotor = new SparkFlex(TurretConstants.TURRET_ID, MotorType.kBrushless);
    turretConfig = new SparkFlexConfig();

    turretEncoder = turretMotor.getEncoder();
    turretController = turretMotor.getClosedLoopController();

    turretConfig
      .smartCurrentLimit(60)
      .idleMode(IdleMode.kBrake)
      .inverted(false);

    turretConfig.encoder
      .positionConversionFactor(TurretConstants.TURRET_POSITION_CONVERSION);

    turretConfig.closedLoop
      .p(TurretConstants.TURRET_PID_VALUES[0])
      .i(TurretConstants.TURRET_PID_VALUES[1])
      .d(TurretConstants.TURRET_PID_VALUES[2])
      .positionWrappingEnabled(false);

    turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    manual = true;
  }

  /**Rotates a the turret to an angle when not on manual control; otherwise, it is controlled with a Joystick */
  public void turretToAngle(double goal, CustomXboxController controller){
    if(!manual){
      double currentPosition = turretEncoder.getPosition();
      double convertedGoal = currentPosition + goal % 360;

      if(convertedGoal > TurretConstants.TURRET_MAX_ANGLE || convertedGoal < TurretConstants.TURRET_MIN_ANGLE)
        convertedGoal = (convertedGoal > TurretConstants.TURRET_MAX_ANGLE) ? convertedGoal - 360 : convertedGoal + 360;
      
      turretController.setSetpoint(convertedGoal, ControlType.kPosition);
    }
    else
      manualTurret(controller);
  }

  /**Controls the turret with joystick inputs */
  public void manualTurret(CustomXboxController controller){
    double speed = controller.getRawAxis(4);
    if(turretEncoder.getPosition() >= TurretConstants.TURRET_MAX_ANGLE -10 && speed > 0)
      stop();

    else if(turretEncoder.getPosition() <= TurretConstants.TURRET_MIN_ANGLE +10 && speed < 0)
      stop();
    
    else
      turretMotor.set(speed);
  }

  /**Sets the turret motor speed to 0 */
  public void stop(){
    turretMotor.set(0);
  }

  /**Toggles whether or not the turret is manual */
  public void toggleManual(){
    manual = !manual;
  }

  /**
   * @return Y coordinate of the turret on the field
   */
  public double getTurretY(){
    double h = swerve.getPose2d().getY(); //Robot Y coordinate
    double k = swerve.getPose2d().getX(); //Robot X coordinate
    double robotAngle = ((swerve.getGyro().getYaw() % 360.0) + 360) % 360; //Robot rotation
    double y = h + TurretConstants.TURRET_OFFSET_Y;
    double x = k + TurretConstants.TURRET_OFFSET_X;
    return h +((y-h)*Math.cos(robotAngle)) - ((x-k) * Math.sin(robotAngle));
  }

  /**
   * @return X coordinate of the turret on the field
   */
  public double getTurretX(){
    double h = swerve.getPose2d().getY(); //Robot y coordinate
    double k = swerve.getPose2d().getX(); //Robot x coordinate
    double robotAngle = ((swerve.getGyro().getYaw() % 360.0) + 360) % 360; //Robot rotation in degrees
    double y = h + TurretConstants.TURRET_OFFSET_Y; //Original y coordinate of turret relative to robot center y coordinate
    double x = k + TurretConstants.TURRET_OFFSET_X; //Original x coordinate of turret relative to robot center x coordinate
    return k +((y-h)*Math.sin(robotAngle)) + ((x-k) * Math.cos(robotAngle));
  }

  /**
   * @param pose Pose to get the angle to
   * @return angle to the pose
   */
  public double getAngleToPose(Pose2d pose){
    double x = pose.getX() - getTurretX();
    double y = pose.getY() - getTurretY();
    return ((Math.atan(x/y))+360)%360;
  }

  /**puts the data for the turret on smartdashboard*/
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    builder.addDoubleProperty("Turret position", () -> turretEncoder.getPosition(), null);
    builder.addDoubleProperty("Turret goal", () -> turretController.getSetpoint(), null);
    builder.addDoubleProperty("Turret X", () -> getTurretX(), null);
    builder.addDoubleProperty("Turret Y", () -> getTurretY(), null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
