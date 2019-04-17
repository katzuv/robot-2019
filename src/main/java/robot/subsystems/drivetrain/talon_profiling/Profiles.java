package robot.subsystems.drivetrain.talon_profiling;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;

import robot.subsystems.drivetrain.Drivetrain;

public class Profiles {

    public static BufferedTrajectoryPointStream toRocketLeft;
    public static BufferedTrajectoryPointStream toRocketRight;

    public static BufferedTrajectoryPointStream toRocketLeft2;
    public static BufferedTrajectoryPointStream toRocketRight2;

    static {
        toRocketLeft = Drivetrain.loadTrajectoryFromCSV("rocket_left.csv");
        toRocketRight = Drivetrain.loadTrajectoryFromCSV("rocket_right.csv");

        toRocketLeft2 = Drivetrain.loadTrajectoryFromCSV("test6_left.csv");
        toRocketRight2 = Drivetrain.loadTrajectoryFromCSV("test6_right.csv");
    }

}
