package robot.subsystems.cargo_intake;

import static robot.Robot.isRobotA;

public class Ports {
    final static int IntakeMotor = 10;
    final static int WristMotor = 9;
    final static int proximitySensor = isRobotA ? 3 : 0;
}
