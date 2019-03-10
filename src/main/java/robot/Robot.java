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
import robot.subsystems.climb.Climb;
import robot.subsystems.cargo_intake.CargoIntake;
import robot.subsystems.drivetrain.Drivetrain;
import robot.subsystems.drivetrain.pure_pursuit.Constants;
import robot.subsystems.drivetrain.pure_pursuit.Path;
import robot.subsystems.drivetrain.pure_pursuit.PurePursue;
import robot.subsystems.drivetrain.pure_pursuit.Waypoint;
import robot.subsystems.elevator.Elevator;
import robot.subsystems.hatch_intake.HatchIntake;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    public static final Climb climb = new Climb();
    public static final Elevator elevator = new Elevator();
    public static final Drivetrain drivetrain = new Drivetrain();
    public static final HatchIntake hatchIntake = new HatchIntake();
    public static final CargoIntake cargoIntake = new CargoIntake();
    public static final Compressor compressor = new Compressor(1);
    public static AHRS navx = new AHRS(SPI.Port.kMXP);


    public static OI m_oi;
    public final static boolean isRobotA = false;

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
        climb.resetEncoders();
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
        climb.executePreventBreak();

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
        navx.reset();
        drivetrain.resetLocation();
        drivetrain.resetEncoders();
        elevator.resetEncoders();


        // String autoSelected = SmartDashboard.getString("Auto Selector","Default"); switch(autoSelected) { case "My Auto": autonomousCommand = new MyAutoCommand(); break; case "Default Auto": default: autonomousCommand = new ExampleCommand(); break; }
        // schedule the autonomous command (example)
        m_autonomousCommand = m_chooser.getSelected();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.start();
        }

        //Create the path and points.
        Path path = new Path();
        path.appendWaypoint(new Waypoint(0, 0));
        path.appendWaypoint(new Waypoint(0, 6));
        //Generate the path to suit the pure pursuit.
        path.generateAll(Constants.WEIGHT_DATA, Constants.WEIGHT_SMOOTH, Constants.TOLERANCE, Constants.MAX_ACCEL, Constants.MAX_PATH_VELOCITY);
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
        compressor.stop();
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
        SmartDashboard.putBoolean("Climb: isClosed", climb.areAllLegsUp());
        SmartDashboard.putNumber("Elevator: height - ticks", elevator.getTicks());
        SmartDashboard.putNumber("Elevator: height - meters", elevator.getHeight());
        SmartDashboard.putNumber("Drivetrain: navx angle", navx.getAngle());
        SmartDashboard.putNumber("Drivetrain: left distance", drivetrain.getLeftDistance());
        SmartDashboard.putNumber("Drivetrain: right distance", drivetrain.getRightDistance());
        SmartDashboard.putNumber("Cargo intake: proximity value", cargoIntake.getProximityVoltage());
        SmartDashboard.putNumber("Cargo intake: wrist angle", cargoIntake.getWristAngle());
        SmartDashboard.putNumber("Elevator: speed", elevator.getSpeed());
        SmartDashboard.putString("Drivetrain: location", String.format("%.4f %.4f", drivetrain.currentLocation.getX(), drivetrain.currentLocation.getY()));
        SmartDashboard.putNumber("Climb: BL height", climb.getLegBLHeight());
        SmartDashboard.putNumber("Climb: BR height", climb.getLegBRHeight());
        SmartDashboard.putNumber("Climb: FL height", climb.getLegFLHeight());
        SmartDashboard.putNumber("Climb: FR height", climb.getLegFRHeight());
    }
}