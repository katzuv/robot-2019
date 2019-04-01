package robot.subsystems.drivetrain.ramsete;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;

import java.util.ArrayList;
import java.util.List;

public class Paths {
    static {
        List<Pose2d> rightHabToRightCargoShip = new ArrayList<>();
    rightHabToRightCargoShip.add(new Pose2d(LengthKt.getFeet(16.28), LengthKt.getFeet(12.607), Rotation2dKt.getDegree(180)));
    }
}
