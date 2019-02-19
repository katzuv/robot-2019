/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.POVButton;
import robot.subsystems.cargo_intake.Constants;
import robot.subsystems.cargo_intake.commands.GripperControl;
import robot.subsystems.cargo_intake.commands.WristTurn;
import robot.subsystems.drivetrain.pure_pursuit.Path;
import robot.subsystems.drivetrain.pure_pursuit.PurePursue;
import robot.subsystems.drivetrain.pure_pursuit.Waypoint;
import robot.subsystems.elevator.commands.ElevatorCommand;
import robot.subsystems.hatch_intake.commands.Gripper;
import robot.subsystems.hatch_intake.commands.GripperTransportation;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

    public static final double WRIST_ROTATE_RATE = 5;
    /**
     * The rate at which the lift will goes down with the xbox joystick.
     */
    public static final double DOWN_SPEED_RATE = 0.08;
    /**
     * The rate at which the lift will goes up with the xbox joystick.
     */
    public static final double UP_SPEED_RATE = 0.08;
    /**
     * The Y value area in which the xbox joystick won't make the lift move.
     */
    public static double XBOX_JOYSTICK_DEAD_BAND = 0;

    public static Joystick leftStick = new Joystick(0);
    public static Joystick rightStick = new Joystick(1);
    public static XboxController xbox = new XboxController(2);
    public static Button a = new JoystickButton(xbox, 1);
    public static Button b = new JoystickButton(xbox, 2);
    public static Button x = new JoystickButton(xbox, 3);
    public static Button y = new JoystickButton(xbox, 4);
    public static Button lb = new JoystickButton(xbox, 5);
    public static Button rb = new JoystickButton(xbox, 6);
    public static Button select = new JoystickButton(xbox, 7);
    public static Button start = new JoystickButton(xbox, 8);
    public static Button ls = new JoystickButton(xbox, 9);
    public static Button rs = new JoystickButton(xbox, 10);
    public static Button povd = new POVButton(xbox, 180);
    public static Button povr = new POVButton(xbox, 90);
    public static Button povl = new POVButton(xbox, 270);

    public static Button lsMid = new JoystickButton(leftStick, 3);
    public static Button lsBottom = new JoystickButton(leftStick, 2);
    public static int left_x_stick = 0;
    public static int left_y_stick = 1;
    public static int left_trigger = 2;
    public static int right_trigger = 3;
    public static int right_x_stick = 4;
    public static int right_y_stick = 5;


    public OI() {
        povd.whenPressed(new ElevatorCommand(0));
        povl.whenPressed(new ElevatorCommand(0.78));
        povr.whenPressed(new ElevatorCommand(1.4));

        rb.whileHeld(new GripperControl(Constants.GRIPPER_SHOOT_SPEED));
        start.whileHeld(new GripperControl(Constants.GRIPPER_INTAKE_SPEED));

        a.whenPressed(new WristTurn(Constants.WRIST_ANGLES.INITIAL));
        b.whenPressed(new WristTurn(Constants.WRIST_ANGLES.UP));
        x.whenPressed(new WristTurn(Constants.WRIST_ANGLES.INTAKE));
        y.whenPressed(new WristTurn(Constants.WRIST_ANGLES.SHOOTING));


        select.whenPressed(new GripperTransportation());
        lb.whenPressed(new Gripper());

        Path drive = new Path(
                new Waypoint(0,0),
                new Waypoint(0,3),
                new Waypoint(-1, 4));
        drive.generateAll(robot.subsystems.drivetrain.pure_pursuit.Constants.WEIGHT_DATA,
                robot.subsystems.drivetrain.pure_pursuit.Constants.WEIGHT_SMOOTH,
                robot.subsystems.drivetrain.pure_pursuit.Constants.TOLERANCE,
                robot.subsystems.drivetrain.pure_pursuit.Constants.MAX_ACCEL,
                robot.subsystems.drivetrain.pure_pursuit.Constants.MAX_PATH_VELOCITY);

        lsMid.whenPressed(new PurePursue(
                drive, robot.subsystems.drivetrain.pure_pursuit.Constants.LOOKAHEAD_DISTANCE,
                robot.subsystems.drivetrain.pure_pursuit.Constants.kV,
                robot.subsystems.drivetrain.pure_pursuit.Constants.kA,
                robot.subsystems.drivetrain.pure_pursuit.Constants.kP,
                true,
                false));


        lsBottom.whenPressed(new PurePursue(
                new Path(
                        new Waypoint(0,0),
                        new Waypoint(1,1)
                ), robot.subsystems.drivetrain.pure_pursuit.Constants.LOOKAHEAD_DISTANCE,
                robot.subsystems.drivetrain.pure_pursuit.Constants.kV,
                robot.subsystems.drivetrain.pure_pursuit.Constants.kA,
                robot.subsystems.drivetrain.pure_pursuit.Constants.kP,
                false,
                false));

    }

    /* instead of defining the joysticks in each default command, all of them call these methods */
    public double leftDriveStick(){ // TODO: might need name refactoring
        return -leftStick.getY();
    }

    public double rightDriveStick(){
        return -rightStick.getY();
    }

    public double WristStick(){
        xbox.getRawAxis(right_y_stick);
    }

    public double ElevatorStick(){
        return -xbox.getRawAxis(left_y_stick);
    }
    // CREATING BUTTONS
    // One type of button is a joystick button which is any button on a
    //// joystick.
    // You create one by telling it which joystick it's on and which button
    // number it is.
    // Joystick stick = new Joystick(port);
    // Button button = new JoystickButton(stick, buttonNumber);

    // There are a few additional built in buttons you can use. Additionally,
    // by subclassing Button you can create custom triggers and bind those to
    // commands the same as any other Button.

    //// TRIGGERING COMMANDS WITH BUTTONS
    // Once you have a button, it's trivial to bind it to a button in one of
    // three ways:

    // Start the command when the button is pressed and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenPressed(new ExampleCommand());

    // Run the command while the button is being held down and interrupt it once
    // the button is released.
    // button.whileHeld(new ExampleCommand());
    // Start the command when the button is released and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenReleased(new ExampleCommand());
}