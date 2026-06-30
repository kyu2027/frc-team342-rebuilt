// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.TurretConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {
  private SparkFlex turretMotor;
  private SparkFlexConfig turretConfig;
  private RelativeEncoder turretEncoder;
  private SparkClosedLoopController turretController;

  private ProfiledPIDController turretPID;
  private SimpleMotorFeedforward turretFF;

  private SwerveDrive swerve;
	private PhotonVision vision;
  private Shooter shooter;

  private double goal;
  
  private SysIdRoutine turretSysIdRoutine;

  /** Creates a new Turret. */
  public Turret(SwerveDrive swerve, PhotonVision vision, Shooter shooter) {
    this.swerve = swerve;
		this.vision = vision;
    this.shooter = shooter;

    turretMotor = new SparkFlex(TurretConstants.TURRET_ID, MotorType.kBrushless);
    turretConfig = new SparkFlexConfig();
    turretEncoder = turretMotor.getEncoder();
    turretController = turretMotor.getClosedLoopController();

    turretPID = new ProfiledPIDController(
      TURRET_PID_VALUES_SLOT1[0],
      TURRET_PID_VALUES_SLOT1[1],
      TURRET_PID_VALUES_SLOT1[2],
      TURRET_CONSTRAINTS
    );

    turretFF = new SimpleMotorFeedforward(
      TURRET_SVA_VALUES[0],
      TURRET_SVA_VALUES[1],
      TURRET_SVA_VALUES[2]
    );

    turretConfig
      .smartCurrentLimit(60)
      .idleMode(IdleMode.kBrake)
      .inverted(false);

    turretConfig.encoder
      .positionConversionFactor(TurretConstants.TURRET_POSITION_CONVERSION);

    turretConfig.closedLoop
      .p(TurretConstants.TURRET_PID_VALUES_SLOT0[0], ClosedLoopSlot.kSlot0)
      .i(TurretConstants.TURRET_PID_VALUES_SLOT0[1], ClosedLoopSlot.kSlot0)
      .d(TurretConstants.TURRET_PID_VALUES_SLOT0[2], ClosedLoopSlot.kSlot0)
			.p(TURRET_PID_VALUES_SLOT1[0], ClosedLoopSlot.kSlot1)
			.i(TURRET_PID_VALUES_SLOT1[1], ClosedLoopSlot.kSlot1)
			.d(TURRET_PID_VALUES_SLOT1[2], ClosedLoopSlot.kSlot1)
      .positionWrappingEnabled(false)
			.allowedClosedLoopError(TURRET_ALLOWED_ERROR, ClosedLoopSlot.kSlot0)
      .allowedClosedLoopError(TURRET_ALLOWED_ERROR, ClosedLoopSlot.kSlot1);

    turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    goal = 0.0;

    turretSysIdRoutine = new SysIdRoutine(
      new Config(
        Volts.of(0.1).per(Second),
        Volts.of(0.5),
        Seconds.of(30)
      ),
      new Mechanism((volts) -> turretMotor.setVoltage(volts.in(Volts)), null, this)
    );
  }

  /**Tells the turret to rotate to the angle to the pose from a lookahead pose.
   * 
   * @param pose The pose to get the angle to.
   */
  public void trackLookAheadPose(Pose2d pose) {
		double safeGoal = MathUtil.clamp(angleToLookAheadPose(pose), TURRET_MIN_ANGLE, TURRET_MAX_ANGLE);
    goal = safeGoal;

		// if(turretEncoder.getPosition() < -40) {
		// 	turretController.setSetpoint(goal, ControlType.kPosition, ClosedLoopSlot.kSlot1);
		// }else if(turretEncoder.getPosition() > goal) {
    // 	turretController.setSetpoint(goal, ControlType.kPosition, ClosedLoopSlot.kSlot0);
		// }else{
		// 	turretController.setSetpoint(goal, ControlType.kPosition, ClosedLoopSlot.kSlot1);
		// }

    turretPID.setGoal(goal);

    double pidInput = turretPID.calculate(turretEncoder.getPosition());
    double ffInput = turretFF.calculate(turretPID.getSetpoint().velocity);
    double voltInput = MathUtil.clamp(pidInput + ffInput, -12, 12);

    turretMotor.setVoltage(voltInput);
  }

  /**Turns the turret to a setpoint.
   * 
   * @param setpoint The angle to turn the turret to.
   */
  public void turnTurret(double setpoint) {
		if(turretEncoder.getPosition() < -40) {
			turretController.setSetpoint(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot1);
		}else if(Math.abs(turretEncoder.getPosition()) > setpoint) {
    	turretController.setSetpoint(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
		}else if(Math.abs(turretEncoder.getPosition()) < setpoint) {
			turretController.setSetpoint(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot1);
		}
  }

  /**Sets the turret motor speed to 0 */
  public void stop(){
    turretMotor.set(0);
  }

  /**Returns a command that runs the system identification routine for the turret.
   * 
   * @return The command that will run the turret system identification routine.
   */
  public Command runTurretSysId() {
    return Commands.sequence(
      turretSysIdRoutine
        .quasistatic(Direction.kForward)
        .until(() -> atDesiredPosition(TURRET_MAX_ANGLE)),
      turretSysIdRoutine
        .quasistatic(Direction.kReverse)
        .until(() -> atDesiredPosition(TURRET_MIN_ANGLE)),
      turretSysIdRoutine
        .dynamic(Direction.kForward)
        .until(() -> atDesiredPosition(TURRET_MAX_ANGLE)),
      turretSysIdRoutine
        .dynamic(Direction.kReverse)
        .until(() -> atDesiredPosition(TURRET_MIN_ANGLE))
    );
  }

  /**Checks if the turret is at the desired position.
   * 
   * @param position The desired position.
   * @return Returns {@code true} if at the desired position, returns {@code false} otherwise.
   */
  public boolean atDesiredPosition(double position) {
    return Math.abs(position - turretEncoder.getPosition()) < TURRET_ALLOWED_ERROR;
  }

  /**Gets the lookahead poses for the robot and the turret.
   * 
   * @return An array containing the lookahead poses for the robot and turret.
   */
  public Pose2d[] getLookAheadPoses() {
    Pose2d currentPose = swerve.getPose2d();

    double xDisplacement = swerve.getChassisSpeeds().vxMetersPerSecond * shooter.getFlightTime(vision.getDistanceToHub(currentPose));
    double yDisplacement = swerve.getChassisSpeeds().vyMetersPerSecond * shooter.getFlightTime(vision.getDistanceToHub(currentPose));

    double displacedX = currentPose.getX() + xDisplacement;
    double displacedY = currentPose.getY() + yDisplacement;

    Pose2d nextRobotPose = new Pose2d(displacedX, displacedY, new Rotation2d(currentPose.getRotation().getRadians()));
    Pose2d nextTurretPose = nextRobotPose.plus(TURRET_OFFSET);

    Pose2d[] lookAheadPoses = {nextRobotPose, nextTurretPose};

    return lookAheadPoses;
  }

  /**Gets the angle to the given pose from the lookahead pose of the turret.
   * 
   * @param pose The pose to get the angle to.
   * @return The angle to the given pose from the lookahead pose of the turret.
   */
  public double angleToLookAheadPose(Pose2d pose) {
    double yDistance = pose.getY() - getLookAheadPoses()[1].getY();
    double xDistance = pose.getX() - getLookAheadPoses()[1].getX();

    Rotation2d angleToTarget = Rotation2d.fromRadians(Math.atan2(yDistance, xDistance));

    double ccwDesiredTurretAngle = -(180 + (MathUtil.inputModulus(angleToTarget.getDegrees(), -180, 180) - MathUtil.inputModulus(getLookAheadPoses()[0].getRotation().getDegrees(), -180, 180)));
    double cwDesiredTurretAngle = 180 - (MathUtil.inputModulus(angleToTarget.getDegrees(), -180, 180) - MathUtil.inputModulus(getLookAheadPoses()[0].getRotation().getDegrees(), -180, 180));

    double ccwAngleDistance = Math.abs(0 - ccwDesiredTurretAngle);
    double cwAngleDistance = Math.abs(0 - cwDesiredTurretAngle);

    if(ccwDesiredTurretAngle < TURRET_MIN_ANGLE && cwDesiredTurretAngle < TURRET_MAX_ANGLE) {
      goal = cwDesiredTurretAngle;
    }else if(ccwDesiredTurretAngle > TURRET_MIN_ANGLE && cwDesiredTurretAngle > TURRET_MAX_ANGLE) {
      goal = ccwDesiredTurretAngle;
    }else if(ccwDesiredTurretAngle > TURRET_MIN_ANGLE && cwDesiredTurretAngle < TURRET_MAX_ANGLE) {
      goal = ccwAngleDistance < cwAngleDistance ? ccwDesiredTurretAngle : cwDesiredTurretAngle;
    }else{
      goal = turretEncoder.getPosition();
    }

    return goal;
  }

  /**puts the data for the turret on smartdashboard*/
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    builder.addDoubleProperty("Turret Encoder Position", () -> turretEncoder.getPosition(), null);
    builder.addDoubleProperty("Turret Turning Velocity", () -> turretEncoder.getVelocity(), null);
    builder.addDoubleProperty("Turret Voltage Draw", () -> turretMotor.getAppliedOutput() * turretMotor.getBusVoltage(), null);
    builder.addDoubleProperty("Turret PID Setpoint", () -> turretController.getSetpoint(), null);
    builder.addDoubleProperty("Turret Assumed Goal", () -> this.goal, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    trackLookAheadPose(vision.getHubCenterPose2d());
  }
}
