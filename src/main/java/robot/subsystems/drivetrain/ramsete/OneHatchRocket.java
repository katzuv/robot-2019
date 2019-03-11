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
//        addParallel(new ElevatorCommand(height));
        List<Pose2d> toRocket = new ArrayList<>();
        toRocket.add(new Pose2d(LengthKt.getFeet(12.426), LengthKt.getFeet(4.632), Rotation2dKt.getDegree(150)));
        toRocket.add(new Pose2d(LengthKt.getFeet(16.912), LengthKt.getFeet(2.106), Rotation2dKt.getDegree(150)));
        addSequential(new DrivePathVision(toRocket, true, true, 0, 0, false));
//        addSequential(new HatchScoring(height, false));
    }

}