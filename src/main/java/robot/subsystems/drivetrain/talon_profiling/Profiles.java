package robot.subsystems.drivetrain.talon_profiling;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;

import robot.subsystems.drivetrain.Drivetrain;

public class Profiles {

    public static BufferedTrajectoryPointStream toNearRocketLeft;
    public static BufferedTrajectoryPointStream toNearRocketRight;

    static {
        toNearRocketLeft = Drivetrain.loadTrajectoryFromCSV("near_rocket_left.csv");
        toNearRocketRight = Drivetrain.loadTrajectoryFromCSV("near_rocket_right.csv");
    }


}
