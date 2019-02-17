package robot.subsystems.drivetrain.commands;

import robot.subsystems.drivetrain.Constants;

public class TurnByAngle2 extends DriveToPosition {
    public TurnByAngle2(double angle){
        super(Math.toRadians(angle) * Constants.ROBOT_WIDTH / 2,- Math.toRadians(angle) * Constants.ROBOT_WIDTH / 2);
    }

}
