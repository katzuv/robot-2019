/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.team254.lib.physics.DifferentialDrive;
import edu.wpi.first.wpilibj.Notifier;
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
import org.ghrobotics.lib.subsystems.drive.TrajectoryTrackerOutput;
import robot.Robot;
import robot.subsystems.drivetrain.commands.JoystickDrive;
import robot.utilities.Point;

/**
 * Drivetrain subsystem for the 2019 robot 'GENESIS'
 *
 * this subsystem holds most of the code for the falcon 5190 kotlin library and its implementation.
 */

public class Drivetrain extends Subsystem {
    private final TalonSRX leftMaster = new TalonSRX(Ports.leftMaster);
    private final VictorSPX leftSlave1 = new VictorSPX(Ports.leftSlave1);
    private final VictorSPX leftSlave2 = new VictorSPX(Ports.leftSlave2);
    private final TalonSRX rightMaster = new TalonSRX(Ports.rightMaster);
    private final VictorSPX rightSlave1 = new VictorSPX(Ports.rightSlave1);
    private final VictorSPX rightSlave2 = new VictorSPX(Ports.rightSlave2);
    public Point currentLocation = new Point(0, 0);

    /**
     * Falcon class. can be called to read the robots position, it stores the robot angle, location and previous location.
     * this class can do many things, and for the full list of methods see the documentation HERE []
     *
     * some of the methods in this kotlin class:
     * .reset(location) ~ sets the robot location (location is a kotlin class aswell)
     * .getRobotPosition() ~ returns a kotlin point, storing all the robots position values.
     * .setRobotX(),.setRobotY(),.setRobotHeading() ~ update the robots position and rotation.
     *
     */
    public Localization localization = new TankEncoderLocalization(
            () -> Rotation2dKt.getDegree(getAngle()),
            () -> LengthKt.getMeter(getLeftDistance()),
            () -> LengthKt.getMeter(getRightDistance())
    );

    /**
     * Falcon class. is in charge of the trajectory following, using the constants defined beforehand.
     * Full documentation HERE []
     *
     * i actually have no idea what this does, but by feeding it a robot position in the vision class,
     * the trajectory is fed to the falcon dashboard, and to the vision path drive.
     */
    public TrajectoryTracker trajectoryTracker = new RamseteTracker(Constants.kBeta, Constants.kZeta);

    public Drivetrain() {
        leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        leftMaster.setSensorPhase(Constants.LEFT_ENCODER_REVERSED);
        rightMaster.setSensorPhase(Constants.RIGHT_ENCODER_REVERSED);

        /* config the motion magic speeds of the drivetrain */
        leftMaster.configMotionCruiseVelocity(convertLeftDistanceToTicks(Constants.MOTION_CRUISE_VELOCITY) / 10);
        rightMaster.configMotionCruiseVelocity(convertRightDistanceToTicks(Constants.MOTION_CRUISE_VELOCITY) / 10);
        leftMaster.configMotionAcceleration(convertLeftDistanceToTicks(Constants.MOTION_ACCELERATION) / 10);
        rightMaster.configMotionAcceleration(convertLeftDistanceToTicks(Constants.MOTION_ACCELERATION) / 10);

            rightMaster.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
            rightMaster.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);

        leftSlave1.follow(leftMaster);
        leftSlave2.follow(leftMaster);
        rightSlave1.follow(rightMaster);
        rightSlave2.follow(rightMaster);

        rightMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);
        leftMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);

        /* make sure the drivetrain always runs at the same amount of volts, even if the battery voltage changes */
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

        /**
         * WPILIB class. makes the localization update run in periodically
         * documentation for those interested:
         * http://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/Notifier.html
         */
        Notifier localizationNotifier = new Notifier(
                () -> localization.update()
        );

        //Set the delay between each update of the method
        localizationNotifier.startPeriodic(1.0 / 100.0); //TODO: isn't 100Hz very load intensive? most roborio loops run at 50Hz
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
        Pose2d robotLocationBeforeReset = localization.getRobotPosition(); //make sure the robots position doesn't move because of the reset encoders.
        leftMaster.setSelectedSensorPosition(0, 0, Constants.TALON_RUNNING_TIMEOUT_MS);
        rightMaster.setSelectedSensorPosition(0, 0, Constants.TALON_RUNNING_TIMEOUT_MS);
        localization.reset(robotLocationBeforeReset);
    }

    public Pose2d getRobotPosition() {
        return localization.getRobotPosition();
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
    public void setMotorsToBrake() {
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
    public void setMotorsToCoast() {
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

    private double convertRightTicksToDistance(int ticks) {
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

    /**
     * Update constants from the shuffleboard directly. useful for debugging
     */
    public void updateConstants() {
        Constants.PIDAngularVelocity[0] = getConstant("Vision Drive: turn kp", Constants.PIDAngularVelocity[0]);
        Constants.PIDAngularVelocity[1] = getConstant("Vision Drive: turn ki", Constants.PIDAngularVelocity[1]);
        Constants.PIDAngularVelocity[2] = getConstant("Vision Drive: turn kd", Constants.PIDAngularVelocity[2]);
        Constants.PIDLinearVelocity[0] = getConstant("Vision Drive: linear kp", Constants.PIDLinearVelocity[0]);
        Constants.PIDLinearVelocity[1] = getConstant("Vision Drive: linear ki", Constants.PIDLinearVelocity[1]);
        Constants.PIDLinearVelocity[2] = getConstant("Vision Drive: linear kd", Constants.PIDLinearVelocity[2]);
    }
    private double getConstant(String key, double constant) {
        SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, constant));
        return SmartDashboard.getNumber(key, constant);
    }

    /**
     * Sends robot position to the falcon dashboard.
     */
    private void updateFalconDashboard(){
        LiveDashboard.INSTANCE.setRobotX(localization.getRobotPosition().getTranslation().getX().getFeet());
        LiveDashboard.INSTANCE.setRobotY(localization.getRobotPosition().getTranslation().getY().getFeet());
        LiveDashboard.INSTANCE.setRobotHeading(localization.getRobotPosition().getRotation().getRadian());
    }

    /**
     * Sends current path to the falcon dashboard
     */
    public void updateLiveDashboard() {
        LiveDashboard.INSTANCE.setPathX(trajectoryTracker.getReferencePoint().getState().getState().getPose().getTranslation().getX().getFeet());
        LiveDashboard.INSTANCE.setPathY(trajectoryTracker.getReferencePoint().getState().getState().getPose().getTranslation().getY().getFeet());
        LiveDashboard.INSTANCE.setPathHeading(trajectoryTracker.getReferencePoint().getState().getState().getPose().getRotation().getRadian());
    }

    @Override
    public void periodic() {
        updateFalconDashboard();
    }

    /**
     * Drive with motion magic to a set position.
     *
     * @param distance distance in meters
     */
    public void driveDistance(double distance) {
        leftMaster.set(ControlMode.MotionMagic, convertLeftDistanceToTicks(distance + getLeftDistance()));
        rightMaster.set(ControlMode.MotionMagic, convertRightDistanceToTicks(distance + getRightDistance()));
    }

    /* Unused as of now */

    /**
     * Use the trajectory output directly.
     * @param output
     */
    public void setOutput(TrajectoryTrackerOutput output) {
        setOutputFromDynamics(output.getDifferentialDriveVelocity(), output.getDifferentialDriveAcceleration());
    }

    /**
     * Robot characterization.
     * @param chassisVelocity
     * @param chassisAcceleration
     */
    public void setOutputFromDynamics(DifferentialDrive.ChassisState chassisVelocity, DifferentialDrive.ChassisState chassisAcceleration) {
        DifferentialDrive.DriveDynamics dynamics = Constants.driveModel.solveInverseDynamics(chassisVelocity, chassisAcceleration);
        setOutput(dynamics.getWheelVelocity(), dynamics.getVoltage());
    }

    /**
     * Robot characterization. no idea what this does, but it internal calculations based on the robots characteristics
     * to determine what voltage to give each motor.
     * @param wheelVelocities
     * @param wheelVoltages
     */
    public void setOutput(DifferentialDrive.WheelState wheelVelocities, DifferentialDrive.WheelState wheelVoltages) {
        leftMaster.set(
                ControlMode.Velocity, convertRightDistanceToTicks(wheelVelocities.getLeft() * Constants.driveModel.getWheelRadius()) / 10.0,
                DemandType.ArbitraryFeedForward,
                wheelVoltages.getLeft() / 12
        );

        rightMaster.set(
                ControlMode.Velocity, convertRightDistanceToTicks(wheelVelocities.getRight() * Constants.driveModel.getWheelRadius()) / 10.0,
                DemandType.ArbitraryFeedForward,
                wheelVoltages.getRight() / 12
        );
    }
}