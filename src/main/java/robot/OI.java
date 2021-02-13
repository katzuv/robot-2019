
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import robot.subsystems.command_groups.CargoScoring;
import robot.subsystems.command_groups.RaiseToHatch;
import robot.subsystems.drivetrain.Drivetrain;
import robot.subsystems.drivetrain.commands.JoystickTurn;
import robot.subsystems.elevator.Elevator;
import robot.subsystems.elevator.commands.ElevatorCommand;
import robot.subsystems.hatch_intake.HatchIntake;
import robot.subsystems.hatch_intake.commands.Fangs;
import robot.subsystems.hatch_intake.commands.Flower;
import robot.subsystems.wrist_control.Constants;
import robot.subsystems.wrist_control.GripperWheels;
import robot.subsystems.wrist_control.WristControl;
import robot.subsystems.wrist_control.commands.GripperControl;
import robot.subsystems.wrist_control.commands.WristAndElevatorCommand;
import robot.subsystems.wrist_control.commands.WristTurn;
import robot.utilities.ButtonCombination;
import robot.utilities.CancelAll;
import robot.utilities.ShiftButton;
import robot.utilities.TriggerButton;

import static robot.subsystems.drivetrain.Constants.SLOW_JOYSTICK_SPEED;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

    public static final Elevator elevator = new Elevator();
    public static final Drivetrain drivetrain = new Drivetrain();
    public static final HatchIntake hatchIntake = new HatchIntake();
    public static final WristControl wristControl = new WristControl();
    public static final GripperWheels gripperWheels = new GripperWheels();


    public static final double WRIST_ROTATE_RATE = 20;
    /**
     * The rate at which the lift will goes down with the xbox joystick.
     */
    public static final double DOWN_SPEED_RATE = 0.1;
    /**
     * The rate at which the lift will goes up with the xbox joystick.
     */
    public static final double UP_SPEED_RATE = 0.13;
    /**
     * The Y value area in which the xbox joystick won't make the lift move.
     */
    public static double XBOX_JOYSTICK_DEAD_BAND = 0.08;

    public static Joystick leftStick = new Joystick(0);
    public static Joystick rightStick = new Joystick(1);
    public static XboxController xbox = new XboxController(2);
    public static JoystickButton a = new JoystickButton(xbox, 1);
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

    public static Button RT = new TriggerButton(xbox, GenericHID.Hand.kRight, 0.1);
    public static Button LT = new TriggerButton(xbox, GenericHID.Hand.kLeft, 0.1);

    public static Button lsLeft = new JoystickButton(leftStick, 4);
    public static Button lsRight = new JoystickButton(leftStick, 5);
    public static Button lsMid = new JoystickButton(leftStick, 3);
    public static Button lsBottom = new JoystickButton(leftStick, 2);

    public static Button ul = new JoystickButton(leftStick, 5);
    public static Button dl = new JoystickButton(leftStick, 3);
    public static Button ur = new JoystickButton(leftStick, 6);
    public static Button dr = new JoystickButton(leftStick, 4);

    public static Button trigger = new JoystickButton(leftStick, 1);
    public static Button back_button = new JoystickButton(leftStick, 2);

    public static int left_x_stick = 0;
    public static int left_y_stick = 1;
    public static int left_trigger = 2;
    public static int right_trigger = 3;
    public static int right_x_stick = 4;
    public static int right_y_stick = 5;


    public static Button left_joystick_two = new JoystickButton(leftStick, 2);
    public static Button left_joystick_three = new JoystickButton(leftStick, 3);
    public static Button left_joystick_six = new JoystickButton(leftStick, 6);
    public static Button left_joystick_seven = new JoystickButton(leftStick, 7);
    public static Button left_joystick_eight = new JoystickButton(leftStick, 8);
    public static Button left_joystick_nine = new JoystickButton(leftStick, 9);
    public static Button left_joystick_ten = new JoystickButton(leftStick, 10);
    public static Button left_joystick_eleven = new JoystickButton(leftStick, 11);

    public static Button right_joystick_two = new JoystickButton(rightStick, 2);
    public static Button right_joystick_six = new JoystickButton(rightStick, 6);
    public static Button right_joystick_seven = new JoystickButton(rightStick, 7);
    public static Button right_joystick_eight = new JoystickButton(rightStick, 8);
    public static Button right_joystick_nine = new JoystickButton(rightStick, 9);
    public static Button right_joystick_ten = new JoystickButton(rightStick, 10);
    public static Button right_joystick_eleven = new JoystickButton(rightStick, 11);

    public static ButtonCombination manual_wrist = new ButtonCombination(xbox, 7, 8, 6);
    public static Button left_joystick_four = new JoystickButton(leftStick, 4);
    public static Button left_joystick_five = new JoystickButton(leftStick, 5);

    public OI() {

        left_joystick_two.whileHeld(new JoystickTurn(drivetrain));
        right_joystick_two.whileHeld(new JoystickTurn(drivetrain));

        /*
        Select (7) + POV = reverse cargo
        Y (4) + POV = front cargo
        Start (8) + POV = Hatch scoring
         */
        LT.whileHeld(new GripperControl(gripperWheels, Constants.GRIPPER_SPEED.INTAKE));
        RT.whileHeld(new GripperControl(gripperWheels, Constants.GRIPPER_SPEED.SHIP, true, GenericHID.Hand.kRight));

        //lb.whileHeld(new(Fangs(true,255)); while held fangs command
//        lb.whenPressed(new CargoRubbing(false));

        rb.whenPressed(new SequentialCommandGroup() {
                           {
                               addCommands((new Flower(hatchIntake, true)),
                                       (new WaitCommand(0.2)),
                                       (new Fangs(hatchIntake, true, 0.5)));
                           }
                       }
        );

        a.whenPressed(new Flower(hatchIntake));
        b.whenPressed(new WristTurn(wristControl, Constants.WRIST_ANGLES.INITIAL));
        x.whenPressed(new WristTurn(wristControl, Constants.WRIST_ANGLES.FORWARD));
        y.whenPressed(new WristTurn(wristControl, Constants.WRIST_ANGLES.UNSAFE_HATCHES));
        int cargoButton = 4; // (y)
        int hatchVisionButton = 8; // Start
        int cargoVisionButton = 7; // Select

        povd.whenPressed(
                new ShiftButton(xbox, cargoButton,
                        new CargoScoring(wristControl, elevator, 0, false),
                        new ShiftButton(xbox, cargoVisionButton,
                                null,
                                new ElevatorCommand(elevator, 0.04)
                        )
                )
        ); //If x is held, go to cargo height, and if not, goes down to zero.

        select.whenPressed(new ShiftButton(
                xbox, cargoButton, new WristAndElevatorCommand(elevator, wristControl, Constants.WRIST_ANGLES.FORWARD.getValue(), robot.subsystems.elevator.Constants.ELEVATOR_HEIGHTS.INTAKE_CARGO.getLevelHeight()),
                null
        ));

        povr.whenPressed(
                new ShiftButton(xbox, hatchVisionButton,
                        new RaiseToHatch(elevator, wristControl, robot.subsystems.elevator.Constants.ELEVATOR_HEIGHTS.LEVEL1_HATCH),
                        new ShiftButton(xbox, cargoButton,
                                new CargoScoring(wristControl, elevator, 1, false),
                                new ShiftButton(xbox, cargoVisionButton,
                                        null,
                                        new ElevatorCommand(elevator, robot.subsystems.elevator.Constants.ELEVATOR_HEIGHTS.LEVEL1_HATCH)
                                )
                        )
                )

        );

        povl.whenPressed(
                new ShiftButton(xbox, hatchVisionButton,
                        new RaiseToHatch(elevator, wristControl, robot.subsystems.elevator.Constants.ELEVATOR_HEIGHTS.LEVEL2_HATCH),
                        new ShiftButton(xbox, cargoButton,
                                new CargoScoring(wristControl, elevator, 2, false),
                                new ShiftButton(xbox, cargoVisionButton,
                                        null,
                                        new ElevatorCommand(elevator, robot.subsystems.elevator.Constants.ELEVATOR_HEIGHTS.LEVEL2_HATCH)
                                )
                        )
                )
        );

        povu.whenPressed(
                new ShiftButton(xbox, hatchVisionButton,
                        new RaiseToHatch(elevator, wristControl, robot.subsystems.elevator.Constants.ELEVATOR_HEIGHTS.LEVEL3_HATCH),
                        new ShiftButton(xbox, cargoButton,
                                new CargoScoring(wristControl, elevator, 3, false),
                                new ShiftButton(xbox, cargoVisionButton,
                                        null,
                                        new ElevatorCommand(elevator, robot.subsystems.elevator.Constants.ELEVATOR_HEIGHTS.LEVEL3_HATCH)
                                )
                        )
                )
        );

        trigger.whenPressed(new CancelAll());
        //left_joystick_seven.toggleWhenPressed(new VisionConditionalCommand(new VelocityVisionDrive()));
        //left_joystick_six.toggleWhenPressed(new VisionConditionalCommand(new VisionTakeHatch()));


        /*
        left_joystick_two.whenPressed(new CalibrateLegs());
        left_joystick_eleven.whenPressed(new CloseForwardLegs());
        left_joystick_ten.whenPressed(new SafeCloseBackLegs());
        left_joystick_nine.whenPressed(new RiseToHeightEncoders(robot.subsystems.climb.Constants.LEVEL_THREE_LEG_LENGTH));
        left_joystick_eight.whenPressed(new RiseToHeightEncoders(robot.subsystems.climb.Constants.LEVEL_TWO_LEG_LENGTH));

        manual_wrist.toggleWhenPressed(new RawWristTurn(0.5190, 1));

        left_joystick_four.whenPressed(new ResetWristAngle(0));
        left_joystick_five.whenPressed(new ResetWristAngle(168));
        */

        // Place cargo backward

        /*

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
    public double leftDriveStick() { // TODO: might need name refactoring
        return -SLOW_JOYSTICK_SPEED * leftStick.getY();
    }

    public double rightDriveStick() {
        return -SLOW_JOYSTICK_SPEED * rightStick.getY();
    }

    public double rightSideAxis() {
        return rightStick.getX();
    }

    public double WristStick() {
        return -xbox.getRawAxis(left_y_stick);
    }

    public double ElevatorStick() {
        return -xbox.getRawAxis(right_y_stick);

    }

    public boolean enableElevator() {
        return xbox.getRawButton(10);
    }

    public boolean enableWrist() {
        return xbox.getRawButton(9);
    }

    public boolean getEnableRawWrist() {
        return xbox.getStartButton() && xbox.getBackButton();
    }
}