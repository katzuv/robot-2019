package robot.subsystems.wrist_control;

import static robot.Robot.isRobotA;

public class Ports {
    final static int WristMotor = 9;

    final static int proximitySensor = isRobotA ? 3 : 0;
    final static int IntakeMotor = 10;
}
