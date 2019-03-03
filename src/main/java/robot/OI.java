/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.POVButton;
import robot.auxiliary.Trigger;
import robot.subsystems.cargo_intake.Constants;
import robot.subsystems.cargo_intake.commands.GripperControl;
import robot.subsystems.cargo_intake.commands.WristTurn;

import robot.subsystems.commandGroups.CargoScoring;
import robot.subsystems.commandGroups.HatchScoring;
import robot.subsystems.drivetrain.commands.GamePiecePickup;

import robot.subsystems.elevator.commands.ConditionalElevatorCommand;
import robot.subsystems.elevator.commands.ElevatorCommand;
import robot.subsystems.hatch_intake.commands.CloseBoth;
import robot.subsystems.hatch_intake.commands.Gripper;
import robot.subsystems.hatch_intake.commands.GripperTransportation;
import robot.subsystems.hatch_intake.commands.PlaceHatch;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

    public static final double WRIST_ROTATE_RATE = 20;
    /**
     * The rate at which the lift will goes down with the xbox joystick.
     */
    public static final double DOWN_SPEED_RATE = 0.05;
    /**
     * The rate at which the lift will goes up with the xbox joystick.
     */
    public static final double UP_SPEED_RATE = 0.05;
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
    public static Button povu = new POVButton(xbox, 0);

    public static Button RT = new Trigger(xbox, GenericHID.Hand.kRight);
    public static Button LT = new Trigger(xbox, GenericHID.Hand.kLeft);

    public static Button lsLeft = new JoystickButton(leftStick, 4);
    public static Button lsRight = new JoystickButton(leftStick, 5);
    public static Button lsMid = new JoystickButton(leftStick, 3);
    public static Button lsBottom = new JoystickButton(leftStick, 2);

    public static Button ul = new JoystickButton(leftStick, 5);
    public static Button dl = new JoystickButton(leftStick, 3);
    public static Button ur = new JoystickButton(leftStick, 6);
    public static Button dr = new JoystickButton(leftStick, 4);

    public static Button trigger = new JoystickButton(leftStick,1);
    public static Button back_button = new JoystickButton(leftStick,2);

    public static Button nine = new JoystickButton(leftStick, 8);
    public static Button ten = new JoystickButton(leftStick, 10);
    public static Button twelve = new JoystickButton(leftStick, 12);

    public static int left_x_stick = 0;
    public static int left_y_stick = 1;
    public static int left_trigger = 2;
    public static int right_trigger = 3;
    public static int right_x_stick = 4;
    public static int right_y_stick = 5;


    public OI() {
        if(Robot.driveType == 1) {
            povd.whenPressed(new ElevatorCommand(0));
            povr.whenPressed(new ConditionalElevatorCommand(new CargoScoring(1, false), new HatchScoring(robot.subsystems.elevator.Constants.ELEVATOR_STATES.LEVEL1_HATCH)));
            povl.whenPressed(new ConditionalElevatorCommand(new CargoScoring(2, false), new HatchScoring(robot.subsystems.elevator.Constants.ELEVATOR_STATES.LEVEL2_HATCH)));
            povu.whenPressed(new ConditionalElevatorCommand(new CargoScoring(3, false), new HatchScoring(robot.subsystems.elevator.Constants.ELEVATOR_STATES.LEVEL3_HATCH)));

            RT.whileHeld(new GripperControl(Constants.GRIPPER_SPEED.SHIP));
            LT.whileHeld(new GripperControl(Constants.GRIPPER_SPEED.INTAKE));

            a.whenPressed(new GripperTransportation());
            lb.whenPressed(new Gripper(false));
            rb.whenPressed(new Gripper(true));

            y.whenPressed(new WristTurn(Constants.WRIST_ANGLES.SHIP));
            b.whenPressed(new WristTurn(Constants.WRIST_ANGLES.INITIAL));
            x.whenPressed(new WristTurn(Constants.WRIST_ANGLES.INTAKE));
            //TODO: add right stick to control the cargo intake
            select.whenPressed(new CloseBoth());
        }else if(Robot.driveType ==2) {
            povd.toggleWhenPressed(new ElevatorCommand(0));
            povl.toggleWhenPressed(new ElevatorCommand(0.78));
            povr.toggleWhenPressed(new ElevatorCommand(1.4));

            RT.whileHeld(new GripperControl(Constants.GRIPPER_SPEED.SHIP));
            LT.whileHeld(new GripperControl(Constants.GRIPPER_SPEED.INTAKE));

            a.toggleWhenPressed(new WristTurn(Constants.WRIST_ANGLES.INITIAL));
            b.toggleWhenPressed(new WristTurn(Constants.WRIST_ANGLES.UP));
            x.toggleWhenPressed(new WristTurn(Constants.WRIST_ANGLES.INTAKE));
            y.whenPressed(new WristTurn(Constants.WRIST_ANGLES.SHIP));


            select.whenPressed(new GripperTransportation());
            lb.whenPressed(new Gripper());
        }
        else if(Robot.driveType == 3){
            povd.whenPressed(new ElevatorCommand(0));
            povl.whenPressed(new ElevatorCommand(0.78));
            povr.whenPressed(new ElevatorCommand(1.3));

            povu.whenPressed(new ElevatorCommand(1.59));
            trigger.whileHeld(new GripperControl(Constants.GRIPPER_SPEED.SHIP));
            ur.whileHeld(new GripperControl(Constants.GRIPPER_SPEED.INTAKE));

            nine.whenPressed(new WristTurn(Constants.WRIST_ANGLES.INITIAL));
            b.whenPressed(new WristTurn(Constants.WRIST_ANGLES.UP));
            ten.whenPressed(new WristTurn(Constants.WRIST_ANGLES.INTAKE));
            twelve.whenPressed(new WristTurn(Constants.WRIST_ANGLES.SHIP));

            dl.whenPressed(new GripperTransportation());
            ul.whenPressed(new Gripper());
        }


        // Place hatch
        //lsLeft.toggleWhenPressed(new HatchScoring(robot.subsystems.elevator.Constants.ELEVATOR_STATES.LEVEL1_HATCH));
        //lsMid.toggleWhenPressed(new HatchScoring(robot.subsystems.elevator.Constants.ELEVATOR_STATES.LEVEL2_HATCH));
        //lsRight.toggleWhenPressed(new HatchScoring(robot.subsystems.elevator.Constants.ELEVATOR_STATES.LEVEL3_HATCH));

        // Place cargo backward

        /*
        Tiood XD

        |||||||
        |||3|||
        |||||||
        |||||||   Rocket
        |||2|||
        |||||||
        |||||||
        |||1|||
        |||||||

        |||||||
        |     |
        |     |    Bay
        |||||||
        |||0|||
        |||||||

         */
        /*
        lsBottom.whenPressed(new CargoScoring(0, true));
        lsLeft.whenPressed(new CargoScoring(1, true));
        lsMid.whenPressed(new CargoScoring(2, true));
        lsRight.whenPressed(new CargoScoring(3, true));

        // Score cargo
        rsBottom.whenPressed(new CargoScoring(0, false));
        rsLeft.whenPressed(new CargoScoring(1, false));
        rsMid.whenPressed(new CargoScoring(2, false));
        rsRight.whenPressed(new CargoScoring(3, false));
        */
    }

    /* instead of defining the joysticks in each default command, all of them call these methods */
    public double leftDriveStick(){ // TODO: might need name refactoring
        if(Robot.driveType==3)
            return -0.5*leftStick.getY()+0.5*leftStick.getZ();
        return -leftStick.getY();
    }

    public double rightDriveStick(){
        if(Robot.driveType==3)
            return -0.5*leftStick.getY()-0.5*leftStick.getZ();
        return -rightStick.getY();
    }

    public double WristStick(){
        if(Robot.driveType==3){
            if(leftStick.getRawButton(9))
                return -0.5;
            else if(leftStick.getRawButton(7))
                return 0.5;
            return 0;
        }
        return -xbox.getRawAxis(left_y_stick);
    }

    public double ElevatorStick() {
        if(Robot.driveType==3)
            return -leftStick.getRawAxis(3);
        return -xbox.getRawAxis(right_y_stick);

    }
    public boolean enableElevator() {
        if(Robot.driveType==3)
            return leftStick.getRawButton(11);
        return xbox.getRawButton(10);
    }


    public boolean enableWrist() {
        if(Robot.driveType==3)
            return true;
        return xbox.getRawButton(9);
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