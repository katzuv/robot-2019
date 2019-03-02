/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d;
import org.ghrobotics.lib.mathematics.twodim.trajectory.TrajectoryGeneratorKt;
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.CentripetalAccelerationConstraint;
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.TimingConstraint;
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.VelocityLimitRegionConstraint;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.AccelerationKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;
import robot.subsystems.cargo_intake.CargoIntake;
import robot.subsystems.drivetrain.Drivetrain;
import robot.subsystems.drivetrain.pure_pursuit.Constants;
import robot.subsystems.drivetrain.pure_pursuit.Path;
import robot.subsystems.drivetrain.pure_pursuit.Waypoint;
import robot.subsystems.drivetrain.ramsete.DrivePathNew;
import robot.subsystems.drivetrain.ramsete.DriveWithVision;
import robot.subsystems.drivetrain.ramsete.HatchAuto;
import robot.subsystems.drivetrain.ramsete.SimpleVisionDrive;
import robot.subsystems.drivetrain.ramsete.VisionTarget;
import robot.subsystems.elevator.Elevator;
import robot.subsystems.hatch_intake.HatchIntake;

import java.util.ArrayList;
import java.util.List;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    public static final Elevator elevator = new Elevator();
    public static final Drivetrain drivetrain = new Drivetrain();
    public static final HatchIntake hatchIntake = new HatchIntake();
    public static final CargoIntake cargoIntake = new CargoIntake();
    public static final Compressor compressor = new Compressor(1);
    public static AHRS navx = new AHRS(SPI.Port.kMXP);
    public static NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("vision");


    public static OI m_oi;
    public final static boolean isRobotA = true;

    Command m_autonomousCommand;
    SendableChooser<Command> m_chooser = new SendableChooser<>();

    /**
     * Send the match type and number to the "vision" table.
     */
    private static void sendMatchInformation() {
        final DriverStation.MatchType matchType = DriverStation.getInstance().getMatchType();
        if (DriverStation.getInstance().isFMSAttached() && matchType != DriverStation.MatchType.None) {
            final int matchNumber = DriverStation.getInstance().getMatchNumber();
            NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("vision");
            NetworkTableEntry matchData = visionTable.getEntry("match data");
            if (matchType == DriverStation.MatchType.Qualification) {
                matchData.setString("Q" + matchNumber);
            } else if (matchType == DriverStation.MatchType.Elimination) {
                if (matchNumber <= 8) {
                    matchData.setString("QF" + matchNumber);
                } else if (matchNumber <= 12) {
                    matchData.setString("TB-QF" + (matchNumber - 8));
                } else if (matchNumber <= 16) {
                    matchData.setString("SF" + matchNumber);
                } else if (matchNumber <= 18) {
                    matchData.setString("TB-SF" + (matchNumber - 4));
                } else if (matchNumber <= 20) {
                    matchData.setString("F" + matchNumber);
                } else {
                    matchData.setString("TB-F");

                }
            } else {
                matchData.setString("P" + matchNumber);
            }
        }
    }

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        m_oi = new OI();

        //m_chooser.setDefaultOption("Default Auto", new JoystickDrive());
        // chooser.addOption("My Auto", new MyAutoCommand());
        SmartDashboard.putData("Auto mode", m_chooser);
        SmartDashboard.putBoolean("Robot A", isRobotA);
        navx.reset();
        elevator.resetEncoders();
    }

    /**
     * This function is called every robot packet, no matter the mode. Use
     * this for items like diagnostics that you want ran during disabled,
     * autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        addToShuffleboard();

    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     * You can use it to reset any subsystem information you want to clear when
     * the robot is disabled.
     */
    @Override
    public void disabledInit() {

        /**TODO: make it so the motor of the wrist has precentoutput 0 or something along those lines
         * to cancel the motion magic that is currently taking place and will still run if you re enable
         */
    }

    @Override
    public void disabledPeriodic() {
        Scheduler.getInstance().run();
        sendMatchInformation();
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable
     * chooser code works with the Java SmartDashboard. If you prefer the
     * LabVIEW Dashboard, remove all of the chooser code and uncomment the
     * getString code to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional commands to the
     * chooser code above (like the commented example) or additional comparisons
     * to the switch structure below with additional strings & commands.
     */
    @Override
    public void autonomousInit() {
//        navx.reset();
//        drivetrain.resetEncoders();
//        elevator.resetEncoders();
        visionTable.getEntry("direction").setString(drivetrain.isDrivingForward() ? "front" : "back");


        // String autoSelected = SmartDashboard.getString("Auto Selector","Default"); switch(autoSelected) { case "My Auto": autonomousCommand = new MyAutoCommand(); break; case "Default Auto": default: autonomousCommand = new ExampleCommand(); break; }
        // schedule the autonomous command (example)
        m_autonomousCommand = m_chooser.getSelected();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.start();
        }

        //Create the path and points.
        Path path = new Path();
        path.appendWaypoint(new Waypoint(0, 0));
        path.appendWaypoint(new Waypoint(0, 2));
        //Generate the path to suit the pure pursuit.
        path.generateAll(Constants.WEIGHT_DATA, Constants.WEIGHT_SMOOTH, Constants.TOLERANCE, Constants.MAX_ACCEL, Constants.MAX_PATH_VELOCITY);
//        PurePursue pursue = new PurePursue(path, Constants.LOOKAHEAD_DISTANCE, Constants.kP, Constants.kA, Constants.kV, false, false);
//        System.out.println(path);
//        pursue.start();

        List<Pose2d> list = new ArrayList<>();
//        list.add(new Pose2d(LengthKt.getMeter(0), LengthKt.getMeter(0), new Rotation2d(0)));
//        list.add(new Pose2d(LengthKt.getMeter(1), LengthKt.getMeter(0), new Rotation2d(0)));
//        Path path = new Path(new Point(0, 0), 0, new Point(-1, 2), -90, 0.55);
//        for (int i = 0; i < path.length() - 1; i++) {
//            Waypoint waypoint = path.getWaypoint(i);
//            Waypoint nextPoint = path.getWaypoint(i + 1);
//            Rotation2d angle = Rotation2dKt.getRadian(Math.PI / 2 - Math.atan2(nextPoint.getY() - waypoint.getY(), nextPoint.getX() - waypoint.getX()));
//            list.add(new Pose2d(LengthKt.getMeter(waypoint.getY()), LengthKt.getMeter(waypoint.getX()), angle));
//        }
//        list.add(new Pose2d(LengthKt.getMeter(path.getWaypoint(path.length() - 1).getY()), LengthKt.getMeter(path.getWaypoint(path.length() - 1).getX()), Rotation2dKt.getDegree(-90)));
        boolean reversed = true;
        double angleModifier = 0;
        if (reversed) {
            angleModifier = 180;
        }

//        list.add(new Pose2d(LengthKt.getFeet(5.441), LengthKt.getFeet(9.77), Rotation2dKt.getDegree(angleModifier - 30)));
        list.add(drivetrain.getRobotPosition());
        list.add(new Pose2d(LengthKt.getFeet(7.454), LengthKt.getFeet(9.638), Rotation2dKt.getDegree(angleModifier)));
        list.add(new Pose2d(LengthKt.getFeet(13.368), LengthKt.getFeet(12.4), Rotation2dKt.getDegree(angleModifier - 15)));
//        list.add(new Pose2d(LengthKt.getFeet(5.595), LengthKt.getFeet(9.704), Rotation2dKt.getDegree(angleModifier + 0)));
//        list.add(new Pose2d(LengthKt.getFeet(11.881), LengthKt.getFeet(6.913), Rotation2dKt.getDegree(angleModifier + -55.0)));
//        list.add(new Pose2d(LengthKt.getFeet(17.727), LengthKt.getFeet(1.618), Rotation2dKt.getDegree(angleModifier + -30.0)));

//        list.add(new Pose2d(LengthKt.getFeet(9.201), LengthKt.getFeet(18.52), Rotation2dKt.getDegree(angleModifier + 0)));
//        list.add(new Pose2d(LengthKt.getFeet(18.477), LengthKt.getFeet(14.398), Rotation2dKt.getDegree(angleModifier + 0)));
//
//        double center_angle = Math.toRadians(visionTable.getEntry("tape_angle").getDouble(0));
//        double distance = visionTable.getEntry("tape_distance").getDouble(0);
//        double field_angle = visionTable.getEntry("tape_field_angle").getDouble(0);
//
//        Pose2d robotPose = drivetrain.localization.getRobotPosition();
//
//        Pose2d calculatedPose = new Pose2d(LengthKt.getMeter(Math.abs(Math.cos(center_angle) * distance)), LengthKt.getMeter(-Math.abs(Math.sin(center_angle) * distance)), Rotation2dKt.getDegree(angleModifier + field_angle));
//
//        list.add(robotPose.transformBy(new Pose2d(Length.Companion.getKZero(), Length.Companion.getKZero(), Rotation2dKt.getDegree(angleModifier))));
//        System.out.println(robotPose.transformBy(calculatedPose).getRotation().getDegree());
//        list.add(robotPose.transformBy(calculatedPose));

        List<TimingConstraint<Pose2dWithCurvature>> constraints = new ArrayList<>();
        constraints.add(new CentripetalAccelerationConstraint(AccelerationKt.getAcceleration(LengthKt.getMeter(1.2192))));
        constraints.add(new VelocityLimitRegionConstraint(new Rectangle2d(LengthKt.getFeet(4), LengthKt.getFeet(7), LengthKt.getFeet(8), LengthKt.getFeet(20)), VelocityKt.getVelocity(LengthKt.getFeet(3))));

        TimedTrajectory<Pose2dWithCurvature> trajectory = TrajectoryGeneratorKt.getDefaultTrajectoryGenerator()
                .generateTrajectory(
                        list,
                        constraints,
                        VelocityKt.getVelocity(Length.Companion.getKZero()),
                        VelocityKt.getVelocity(LengthKt.getMeter(0)),
                        VelocityKt.getVelocity(LengthKt.getMeter(0.5)),
                        AccelerationKt.getAcceleration(LengthKt.getMeter(0.5)),
                        reversed,
                        true
                );

        new DriveWithVision(trajectory).start();
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {

        Scheduler.getInstance().run();

    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        cargoIntake.resetSensors(); // TODO: move to auto init. deal with all resets better

        navx.reset();
        drivetrain.resetLocation();
        drivetrain.resetEncoders();
        elevator.resetEncoders();
        navx.reset();
        cargoIntake.resetSensors();
        compressor.start();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();

    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }


    public void addToShuffleboard() {
        SmartDashboard.putNumber("Elevator: height - ticks", elevator.getTicks());
        SmartDashboard.putNumber("Elevator: height - meters", elevator.getHeight());
        SmartDashboard.putNumber("Drivetrain: navx angle", navx.getAngle());
        SmartDashboard.putNumber("Drivetrain: left distance", drivetrain.getLeftDistance());
        SmartDashboard.putNumber("Drivetrain: right distance", drivetrain.getRightDistance());
        SmartDashboard.putNumber("Cargo intake: proximity value", cargoIntake.getProximityVoltage());
        SmartDashboard.putNumber("Cargo intake: wrist angle", cargoIntake.getWristAngle());
        SmartDashboard.putNumber("Elevator: speed", elevator.getSpeed());
        Translation2d robotLocation = drivetrain.getRobotPosition().getTranslation();
        SmartDashboard.putString("Drivetrain: location", String.format("%.4f %.4f", robotLocation.getX().getMeter(), robotLocation.getY().getMeter()));

    }
}