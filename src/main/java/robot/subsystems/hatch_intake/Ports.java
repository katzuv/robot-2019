package robot.subsystems.hatch_intake;

import static robot.Robot.isRobotA;

public class Ports {
    public final static int flowerReverse = isRobotA ? 4 : 1;
    public final static int flowerForward = isRobotA ? 1 : 0;
    public final static int pusherReverse = isRobotA ? 3 : 3;
    public final static int pusherForward = isRobotA ? 2 : 2;

    public final static int proximitySensor = 7;
}