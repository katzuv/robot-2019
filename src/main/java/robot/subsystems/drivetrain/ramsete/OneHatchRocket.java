package robot.subsystems.drivetrain.ramsete;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import robot.subsystems.commandGroups.HatchScoring;
import robot.subsystems.elevator.Constants;
import robot.subsystems.elevator.commands.ElevatorCommand;
import robot.subsystems.hatch_intake.commands.TakeHatch;

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
        toRocket.add(new Pose2d(LengthKt.getFeet(12.973), LengthKt.getFeet(5.491), Rotation2dKt.getDegree(150)));
        toRocket.add(new Pose2d(LengthKt.getFeet(16.661), LengthKt.getFeet(3.177), Rotation2dKt.getDegree(150)));
        addSequential(new DrivePathVision(toRocket, true, true, 0, 0, false));

        //Score hatch
        addSequential(new HatchScoring(height, false));

        List<Pose2d> driveBack = new ArrayList<>();
        driveBack.add(new Pose2d(LengthKt.getFeet(11.677), LengthKt.getFeet(6.412), Rotation2dKt.getDegree(90)));
        addSequential(new DrivePathVision(driveBack, false, false, 0, 0, false));

        List<Pose2d> driveForward = new ArrayList<>();
        driveForward.add(new Pose2d(LengthKt.getFeet(7.922), LengthKt.getFeet(3.2), Rotation2dKt.getDegree(0)));

        addSequential(new DrivePathVision(driveForward, true, false, 0, 0, false));

        List<Pose2d> toLoadingStation = new ArrayList<>();
        toLoadingStation.add(new Pose2d(LengthKt.getFeet(2.4), LengthKt.getFeet(3.2), Rotation2dKt.getDegree(0)));

        addSequential(new DrivePathVision(toLoadingStation, true, true, 0, 0, false));

        addSequential(new TakeHatch());

    }

}
