package robot.subsystems.drivetrain.sandstorm;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import robot.utilities.Utils;

import java.util.ArrayList;
import java.util.Arrays;

class Paths {
    static final TimedTrajectory<Pose2dWithCurvature> RIGHT_HAB_TO_NEAR_ROCKET;
    static final TimedTrajectory<Pose2dWithCurvature> RIGHT_HAB_TO_FAR_ROCKET;

    static final TimedTrajectory<Pose2dWithCurvature> NEAR_ROCKET_TO_LOADING;

    static final TimedTrajectory<Pose2dWithCurvature> LOADING_TO_FAR_ROCKET;
    static{
        RIGHT_HAB_TO_FAR_ROCKET =Utils.generateTrajectory(
                Arrays.asList(
                        new Pose2d(LengthKt.getFeet(6.321), LengthKt.getFeet(9.408), Rotation2dKt.getDegree(0)),
                        new Pose2d(LengthKt.getFeet(9.408), LengthKt.getFeet(5.496), Rotation2dKt.getDegree(15)),
                        new Pose2d(LengthKt.getFeet(25.526), LengthKt.getFeet(4.373), Rotation2dKt.getDegree(30))
                ),0,1,false
        ); //TODO: not tested, placeholder

        RIGHT_HAB_TO_NEAR_ROCKET = Utils.generateTrajectory(
                Arrays.asList(
                        new Pose2d(LengthKt.getFeet(6.321), LengthKt.getFeet(9.408), Rotation2dKt.getDegree(180)),
                        new Pose2d(LengthKt.getFeet(11.68), LengthKt.getFeet(4.789), Rotation2dKt.getDegree(150))
                ), 0, 0, true
        );

        NEAR_ROCKET_TO_LOADING = Utils.generateTrajectory(
                Arrays.asList(
                        new Pose2d(LengthKt.getFeet(13.148), LengthKt.getFeet(2.738), Rotation2dKt.getDegree(0)),
                        new Pose2d(LengthKt.getFeet(7.769), LengthKt.getFeet(2.187), Rotation2dKt.getDegree(0))
                ),0,1,false
        );

        LOADING_TO_FAR_ROCKET =Utils.generateTrajectory(
                Arrays.asList(
                        new Pose2d(LengthKt.getFeet(1.8), LengthKt.getFeet(2.0), Rotation2dKt.getDegree(0)),
                        new Pose2d(LengthKt.getFeet(15.462), LengthKt.getFeet(4.616), Rotation2dKt.getDegree(21)),
                        new Pose2d(LengthKt.getFeet(22.571), LengthKt.getFeet(2.885), Rotation2dKt.getDegree(-90))
                ),0,0,false
        );

    }
}
