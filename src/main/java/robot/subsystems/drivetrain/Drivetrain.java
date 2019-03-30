/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.ghrobotics.lib.debug.LiveDashboard;
import org.ghrobotics.lib.localization.Localization;
import org.ghrobotics.lib.localization.TankEncoderLocalization;
import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker;
import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTracker;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import robot.Robot;
import robot.subsystems.drivetrain.commands.JoystickDrive;
import robot.utilities.Point;

import static robot.Robot.drivetrain;

/**
 * Add your docs here.
 */

public class Drivetrain extends Subsystem {
    private final TalonSRX leftMaster = new TalonSRX(Ports.leftMaster);
    private final VictorSPX leftSlave1 = new VictorSPX(Ports.leftSlave1);
    private final VictorSPX leftSlave2 = new VictorSPX(Ports.leftSlave2);
    private final TalonSRX rightMaster = new TalonSRX(Ports.rightMaster);
    private final VictorSPX rightSlave1 = new VictorSPX(Ports.rightSlave1);
    private final VictorSPX rightSlave2 = new VictorSPX(Ports.rightSlave2);
    public Point currentLocation = new Point(0, 0);

    public Localization localization = new TankEncoderLocalization(
            () -> Rotation2dKt.getDegree(180 + getAngle()),
            () -> LengthKt.getMeter(getLeftDistance()),
            () -> LengthKt.getMeter(getRightDistance())
    );

    public TrajectoryTracker trajectoryTracker = new RamseteTracker(Constants.kBeta, Constants.kZeta);

    public Drivetrain() {
        leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        leftMaster.setSensorPhase(Constants.LEFT_ENCODER_REVERSED);
        rightMaster.setSensorPhase(Constants.RIGHT_ENCODER_REVERSED);

        leftMaster.configMotionCruiseVelocity(convertLeftDistanceToTicks(Constants.MOTION_CRUISE_VELOCITY) / 10);
        rightMaster.configMotionCruiseVelocity(convertRightDistanceToTicks(Constants.MOTION_CRUISE_VELOCITY) / 10);
        leftMaster.configMotionAcceleration(convertLeftDistanceToTicks(Constants.MOTION_ACCELERATION) / 10);
        rightMaster.configMotionAcceleration(convertLeftDistanceToTicks(Constants.MOTION_ACCELERATION) / 10);

        if (!Robot.isRobotA) {
            rightMaster.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
            rightMaster.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
        }

        leftSlave1.follow(leftMaster);
        leftSlave2.follow(leftMaster);
        rightSlave1.follow(rightMaster);
        rightSlave2.follow(rightMaster);

        rightMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);
        leftMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);

        leftMaster.configVoltageCompSaturation(12);
        leftMaster.enableVoltageCompensation(true);
        leftSlave1.configVoltageCompSaturation(12);
        leftSlave1.enableVoltageCompensation(true);
        leftSlave2.configVoltageCompSaturation(12);
        leftSlave2.enableVoltageCompensation(true);
        rightMaster.configVoltageCompSaturation(12);
        rightMaster.enableVoltageCompensation(true);
        rightSlave2.configVoltageCompSaturation(12);
        rightSlave2.enableVoltageCompensation(true);
        rightSlave1.configVoltageCompSaturation(12);
        rightSlave1.enableVoltageCompensation(true);

        leftMaster.setInverted(Constants.LEFT_MASTER_REVERSED);
        leftSlave1.setInverted(Constants.LEFT_SLAVE1_REVERSED);
        leftSlave2.setInverted(Constants.LEFT_SLAVE2_REVERSED);
        rightMaster.setInverted(Constants.RIGHT_MASTER_REVERSED);
        rightSlave1.setInverted(Constants.RIGHT_SLAVE1_REVERSED);
        rightSlave2.setInverted(Constants.RIGHT_SLAVE2_REVERSED);

        setMotorsToCoast();

        leftMaster.config_kP(0, Constants.PIDFLeft[0], Constants.TALON_TIMEOUT_MS);
        leftMaster.config_kI(0, Constants.PIDFLeft[1], Constants.TALON_TIMEOUT_MS);
        leftMaster.config_kD(0, Constants.PIDFLeft[2], Constants.TALON_TIMEOUT_MS);
        leftMaster.config_kF(0, Constants.PIDFLeft[3], Constants.TALON_TIMEOUT_MS);

        rightMaster.config_kP(0, Constants.PIDFRight[0], Constants.TALON_TIMEOUT_MS);
        rightMaster.config_kI(0, Constants.PIDFRight[1], Constants.TALON_TIMEOUT_MS);
        rightMaster.config_kD(0, Constants.PIDFRight[2], Constants.TALON_TIMEOUT_MS);
        rightMaster.config_kF(0, Constants.PIDFRight[3], Constants.TALON_TIMEOUT_MS);

    }

    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new JoystickDrive());
    }

    /**
     * Set the speed for both sides.
     *
     * @param leftSpeed  Speed for the left side
     * @param rightSpeed Speed for the right side
     */
    public void setSpeed(double leftSpeed, double rightSpeed) {
        setLeftSpeed(leftSpeed);
        setRightSpeed(rightSpeed);
    }

    public double getLeftVelocity() {
        return convertLeftTicksToDistance(leftMaster.getSelectedSensorVelocity()) * 10;
    }

    public void setLeftVelocity(double velocity) {
        leftMaster.set(ControlMode.Velocity, convertLeftDistanceToTicks(velocity) / 10.0);
    }

    public double getRightVelocity() {
        return convertRightTicksToDistance(rightMaster.getSelectedSensorVelocity()) * 10;
    }

    public void setRightVelocity(double velocity) {
        rightMaster.set(ControlMode.Velocity, convertRightDistanceToTicks(velocity) / 10.0);
    }

    public void setVelocity(double left, double right) {
        setLeftVelocity(left);
        setRightVelocity(right);
    }

    /**
     * @return the speed of the left side of the Drivetrain
     */
    public double getLeftSpeed() {
        return convertLeftTicksToDistance(leftMaster.getSelectedSensorVelocity(0)) * 10;
    }

    /**
     * Set the speed for the left side.
     *
     * @param speed speed for the motors of the left side
     */
    private void setLeftSpeed(double speed) {
        leftMaster.set(ControlMode.PercentOutput, speed);
    }

    /**
     * @return the speed of the right side of the Drivetrain
     */
    public double getRightSpeed() {
        return convertRightTicksToDistance(leftMaster.getSelectedSensorVelocity(0)) * 10;
    }

    /**
     * Set the speed for the right side.
     *
     * @param speed speed for the motors of the right side
     */
    private void setRightSpeed(double speed) {
        rightMaster.set(ControlMode.PercentOutput, speed);
    }

    /**
     * @return The distance driven on the left side of the robot since the last
     * reset
     */
    public double getLeftDistance() {
        return convertLeftTicksToDistance(leftMaster.getSelectedSensorPosition(0));
    }

    /**
     * @return The distance driven on the right side of the robot since the last
     * reset
     */
    public double getRightDistance() {
        return convertRightTicksToDistance(rightMaster.getSelectedSensorPosition(0));
    }

    public void resetEncoders() {
        Pose2d robotLocationBeforeReset = localization.getRobotPosition();
        leftMaster.setSelectedSensorPosition(0, 0, Constants.TALON_RUNNING_TIMEOUT_MS);
        rightMaster.setSelectedSensorPosition(0, 0, Constants.TALON_RUNNING_TIMEOUT_MS);
        localization.reset(robotLocationBeforeReset);
    }

    public Pose2d getRobotPosition() {
        return localization.getRobotPosition();
    }

    public boolean isDrivingForward() {
        return getLeftSpeed() >= 0 || getRightSpeed() >= 0;
    }

    public void resetLocation() {
        localization.reset(new Pose2d(LengthKt.getFeet(6.321), LengthKt.getFeet(9.408), Rotation2dKt.getDegree(180))); //TODO: make this a constant
    }

    public void resetLocation(Pose2d pose) {
        localization.reset(pose);
    }

    /**
     * Sets all the motors of the drivetrain to a brake neutral mode
     */
    public void setMotorsToBrake(){
        leftMaster.setNeutralMode(NeutralMode.Brake);
        leftSlave1.setNeutralMode(NeutralMode.Brake);
        leftSlave2.setNeutralMode(NeutralMode.Brake);
        rightMaster.setNeutralMode(NeutralMode.Brake);
        rightSlave2.setNeutralMode(NeutralMode.Brake);
        rightSlave1.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * Sets all the motors of the drivetrain to a coast neutral mode
     */
    public void setMotorsToCoast(){
        leftMaster.setNeutralMode(NeutralMode.Coast);
        leftSlave1.setNeutralMode(NeutralMode.Coast);
        leftSlave2.setNeutralMode(NeutralMode.Coast);
        rightMaster.setNeutralMode(NeutralMode.Coast);
        rightSlave2.setNeutralMode(NeutralMode.Coast);
        rightSlave1.setNeutralMode(NeutralMode.Coast);
    }


    /**
     * Convert distance in meters to ticks of the encoder.
     *
     * @param distance height in meters
     * @return ticks of the encoder
     */
    private int convertLeftDistanceToTicks(double distance) {
        return (int) (distance * Constants.LEFT_TICKS_PER_METER);
    }

    private int convertRightDistanceToTicks(double distance) {
        return (int) (distance * Constants.RIGHT_TICKS_PER_METER);
    }

    /**
     * Convert ticks of the encoder to distance in meters.
     *
     * @param ticks ticks of the encoder
     * @return height in meters
     */
    private double convertLeftTicksToDistance(int ticks) {
        return ticks / Constants.LEFT_TICKS_PER_METER;
    }

    private double convertRightTicksToDistance(int ticks){
        return ticks / Constants.RIGHT_TICKS_PER_METER;
    }
    /**
     * returns the robot NAVX yaw angle
     *
     * @return navx yaw angle
     */
    public double getAngle() {
        return -Robot.navx.getAngle();
    }

    /**
     * returns the robot NAVX pitch angle
     *
     * @return navx pitch angle
     */
    public double getPitch() {
        return Robot.navx.getPitch();
    }

    /**
     * returns the robot NAVX roll angle
     *
     * @return navx roll angle
     */
    public double getRoll() {
        return Robot.navx.getRoll();
    }

    public double getYaw() {
        return Robot.navx.getYaw();
    }

    @Override
    public void periodic() {
        localization.update();
        LiveDashboard.INSTANCE.setRobotX(localization.getRobotPosition().getTranslation().getX().getFeet());
        LiveDashboard.INSTANCE.setRobotY(localization.getRobotPosition().getTranslation().getY().getFeet());
        LiveDashboard.INSTANCE.setRobotHeading(localization.getRobotPosition().getRotation().getRadian());
    }

    public Point getCurrentLocation() {
        return new Point(localization.getRobotPosition().getTranslation().getY().getMeter(), localization.getRobotPosition().getTranslation().getX().getMeter());
    }

    public void driveDistance(double distance) {
        leftMaster.set(ControlMode.MotionMagic, convertLeftDistanceToTicks(distance + getLeftDistance()));
        rightMaster.set(ControlMode.MotionMagic, convertRightDistanceToTicks(distance + getRightDistance()));
        System.out.println(getLeftDistance());
        System.out.println(convertLeftDistanceToTicks(distance + getLeftDistance()));
        System.out.println(convertRightDistanceToTicks(getLeftDistance()));
    }

    public void updateConstants() {
        Constants.PIDVisionTurn[0] = getConstant("Vision Drive: turn kp", Constants.PIDVisionTurn[0]);
        Constants.PIDVisionTurn[1] = getConstant("Vision Drive: turn ki", Constants.PIDVisionTurn[1]);
        Constants.PIDVisionTurn[2] = getConstant("Vision Drive: turn kd", Constants.PIDVisionTurn[2]);

    }

    private double getConstant(String key, double constant){
        SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, constant));
        return SmartDashboard.getNumber(key, constant);
    }

    public void updateLiveDashboard() {
        LiveDashboard.INSTANCE.setPathX(trajectoryTracker.getReferencePoint().getState().getState().getPose().getTranslation().getX().getFeet());
        LiveDashboard.INSTANCE.setPathY(trajectoryTracker.getReferencePoint().getState().getState().getPose().getTranslation().getY().getFeet());
        LiveDashboard.INSTANCE.setPathHeading(trajectoryTracker.getReferencePoint().getState().getState().getPose().getRotation().getRadian());

    }
}