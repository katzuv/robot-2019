package robot.subsystems.drivetrain.ramsete;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import robot.subsystems.commandGroups.HatchScoring;
import robot.subsystems.elevator.Constants;
import robot.subsystems.elevator.commands.ElevatorCommand;

import java.util.ArrayList;
import java.util.List;

/**
 *
 */
public class OneHatchRocket extends CommandGroup {

    public OneHatchRocket(Constants.ELEVATOR_STATES height) {
        //Lift elevator before time
        addParallel(new ElevatorCommand(height));
        //Drive to rocket
        List<Pose2d> toRocket = new ArrayList<>();
        toRocket.add(new Pose2d(LengthKt.getFeet(13.052), LengthKt.getFeet(4.284), Rotation2dKt.getDegree(150)));
        toRocket.add(new Pose2d(LengthKt.getFeet(16.996), LengthKt.getFeet(1.976), Rotation2dKt.getDegree(150)));
        addSequential(new DrivePathVision(toRocket, true, true, 0, 0, false));
        //Score hatch
        addSequential(new HatchScoring(height, false));

        //Turn to prepare for loading station path
//        List<Pose2d> turn = new ArrayList<>();
//        turn.add(new Pose2d(LengthKt.getFeet(13.416), LengthKt.getFeet(3.847), Rotation2dKt.getDegree(-90)));
//        addSequential(new DrivePathVision(turn, false, false, 0, 0, false));

    }

}