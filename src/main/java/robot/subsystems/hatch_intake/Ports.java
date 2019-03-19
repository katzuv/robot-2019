package robot.subsystems.hatch_intake;

import static robot.Robot.isRobotA;

public class Ports {
    public final static int flowerReverse = isRobotA ? 3 : 0;
    public final static int flowerForward = isRobotA ? 2 : 3;
    public final static int extensionPlateReverse = isRobotA ? 0 : 1;
    public final static int extensionPlateForward = isRobotA ? 1 : 2;

    public final static int proximitySensor = 7;
}