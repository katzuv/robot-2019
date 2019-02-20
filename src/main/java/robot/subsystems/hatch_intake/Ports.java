package robot.subsystems.hatch_intake;

import static robot.Robot.isRobotA;

public class Ports {
    public final static int gripperReverse = isRobotA ? 2 : 2;
    public final static int gripperForward = isRobotA ? 3 : 3;
    public final static int gripperPlateReverse = isRobotA ? 1 : 1;
    public final static int gripperPlateForward = isRobotA ? 0 : 0;
    public final static int proximitySensor = 7;
}
