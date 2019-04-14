package robot.subsystems.drivetrain.sandstorm;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import robot.subsystems.drivetrain.Constants;
import robot.utilities.Utils;

import java.util.Arrays;

class Paths {
    static final TimedTrajectory<Pose2dWithCurvature> RIGHT_HAB_TO_NEAR_ROCKET;

    static final TimedTrajectory<Pose2dWithCurvature> MIDDLE_HAB_TO_RIGHT_CARGO;


    static final TimedTrajectory<Pose2dWithCurvature> RIGHT_HAB_TO_FAR_ROCKET;

    static final TimedTrajectory<Pose2dWithCurvature> LEFT_HAB_TO_NEAR_ROCKET;

    static {
        RIGHT_HAB_TO_FAR_ROCKET = Utils.generateTrajectory(
                Arrays.asList(
                        new Pose2d(LengthKt.getFeet(6.321), LengthKt.getFeet(9.408), Rotation2dKt.getDegree(0)),
                        new Pose2d(LengthKt.getFeet(9.408), LengthKt.getFeet(5.496), Rotation2dKt.getDegree(15)),
                        new Pose2d(LengthKt.getFeet(25.526), LengthKt.getFeet(4.373), Rotation2dKt.getDegree(30))
                ), 0, 0, false
        ); //TODO: not tested, placeholder

        RIGHT_HAB_TO_NEAR_ROCKET = Utils.generateTrajectory(
                Arrays.asList(
                        new Pose2d(LengthKt.getFeet(6.321), LengthKt.getFeet(9.408), Rotation2dKt.getDegree(180)),
                        new Pose2d(LengthKt.getFeet(12.707), LengthKt.getFeet(4.78), Rotation2dKt.getDegree(150))
                ), 0, 0, true
        );

        LEFT_HAB_TO_NEAR_ROCKET = Utils.generateTrajectory(
                Arrays.asList(
                        new Pose2d(LengthKt.getFeet(6.321), LengthKt.getFeet(17.543), Rotation2dKt.getDegree(180)),
                        new Pose2d(LengthKt.getFeet(12.707), LengthKt.getFeet(22.22), Rotation2dKt.getDegree(-155))
                ), 0, 0, true
        );

        MIDDLE_HAB_TO_RIGHT_CARGO = Utils.generateTrajectory(
                Arrays.asList(
                        new Pose2d(LengthKt.getFeet(6.321), LengthKt.getFeet(13.565), Rotation2dKt.getDegree(180)),
                        new Pose2d(LengthKt.getFeet(13.5), LengthKt.getFeet(12.255), Rotation2dKt.getDegree(180))
                ), 1, 0, true
        );


    }
}
