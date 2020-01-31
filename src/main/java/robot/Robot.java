/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.eclipse.jetty.util.thread.Scheduler;
import robot.subsystems.drivetrain.Drivetrain;
import robot.subsystems.elevator.Elevator;
import robot.subsystems.elevator.commands.ResetElevatorHeight;
import robot.subsystems.hatch_intake.HatchIntake;
import robot.subsystems.hatch_intake.commands.Flower;
import robot.subsystems.wrist_control.GripperWheels;
import robot.subsystems.wrist_control.WristControl;
import robot.subsystems.wrist_control.commands.ResetWristAngle;
import robot.subsystems.wrist_control.commands.WristTurn;
import robot.utilities.MotorIssueDetector;

import java.lang.reflect.Field;
import java.util.Hashtable;
import java.util.Set;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    public static final PowerDistributionPanel pdp = new PowerDistributionPanel();
    public static final Elevator elevator = new Elevator();
    public static final Drivetrain drivetrain = new Drivetrain();
    public static final HatchIntake hatchIntake = new HatchIntake();
    public static final WristControl wristControl = new WristControl();
    public static final GripperWheels gripperWheels = new GripperWheels();
    public static final Compressor compressor = new Compressor(1);
    public static final MotorIssueDetector motorChecker = new MotorIssueDetector(pdp);
    public final static boolean isRobotA = false;
    public final static boolean debug = false;
    public static AHRS navx = new AHRS(SPI.Port.kMXP);
    public static NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("vision");
    public static OI m_oi;

    static {
        NetworkTableInstance.getDefault().setUpdateRate(0.01); //Set update rate to 10ms for vision
    }

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
        resetAll();

        m_oi = new OI();



        SmartDashboard.putBoolean("Robot A", isRobotA);

        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setWhiteBalanceAuto();
        camera.setExposureAuto();
        camera.setBrightness(7);
        camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
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
        compressor.stop();
        addToShuffleboard();
        if (debug) {
            updateDashboardConstants();
        }
        motorChecker.update();

        //SmartDashboard.putBoolean("Wrist: prevented reset", wristControl.preventEncoderJumps());
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     * You can use it to reset any subsystem information you want to clear when
     * the robot is disabled.
     */
    @Override
    public void disabledInit() {
        elevator.ResetSetpoint();
        drivetrain.setMotorsToCoast();
        /**TODO: make it so the motor of the wrist has precentoutput 0 or something along those lines
         * to cancel the motion magic that is currently taking place and will still run if you re enable
         */
    }

    @Override
    public void disabledPeriodic() {
        wristControl.disabledPeriodic();
        CommandScheduler.getInstance().run();
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
        resetAll();
        m_autonomousCommand = m_chooser.getSelected();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }

        drivetrain.setMotorsToBrake();
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        drivetrain.setMotorsToCoast();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        //updateDashboardConstants();
        CommandScheduler.getInstance().run();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }


    public void addToShuffleboard() {
        SmartDashboard.putData(pdp);
        SmartDashboard.putNumber("Elevator: height - centimeters", elevator.getHeight());
        SmartDashboard.putNumber("Wrist: wrist angle", wristControl.getWristAngle());
        SmartDashboard.putBoolean("Flower open", hatchIntake.isFlowerOpen());

        SmartDashboard.putData("navx", navx);
        SmartDashboard.putData("Reset wrist encoders", new ResetWristAngle(0));
        SmartDashboard.putData("Reset wrist to 150 degrees", new ResetWristAngle(150));
        SmartDashboard.putData("reset elevator height", new ResetElevatorHeight());

        if (debug) {
            SmartDashboard.putNumber("Wrist: proximity value", gripperWheels.getProximityVoltage());
            SmartDashboard.putString("Drivetrain: location", String.format("%.4f %.4f", drivetrain.currentLocation.getX(), drivetrain.currentLocation.getY()));
        }
    }

    public void updateDashboardConstants() {
        drivetrain.updateConstants();
        wristControl.updateConstants();

    }

    public void printRunningCommands() {
        try {
            Field field = Scheduler.class.getField("m_commandTable");
            field.setAccessible(true);
            Hashtable table = ((Hashtable) field.get(CommandScheduler.getInstance()));
            Set<Command> commands = table.keySet();
            commands.forEach(c -> System.out.println(c.getName()));
        } catch (IllegalAccessException | NoSuchFieldException e) {
            e.printStackTrace();
        }
    }
    public void resetAll() {
        wristControl.resetSensors();
        elevator.resetEncoders();
        navx.reset();
    }
}