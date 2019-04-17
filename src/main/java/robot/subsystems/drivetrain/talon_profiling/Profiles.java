package robot.subsystems.drivetrain.talon_profiling;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;

import robot.subsystems.drivetrain.Drivetrain;

public class Profiles {

    public static BufferedTrajectoryPointStream toRocketLeft;
    public static BufferedTrajectoryPointStream toRocketRight;

    public static BufferedTrajectoryPointStream toFarRocketLeft;
    public static BufferedTrajectoryPointStream toFarRocketRight;

    public static BufferedTrajectoryPointStream farRocketToLoadingRevLeft;
    public static BufferedTrajectoryPointStream farRocketToLoadingRevRight;

    static {
        toRocketLeft = Drivetrain.loadTrajectoryFromCSV("rocket_left.csv");
        toRocketRight = Drivetrain.loadTrajectoryFromCSV("rocket_right.csv");

        toFarRocketLeft = Drivetrain.loadTrajectoryFromCSV("far_rocket_p1_left.csv");
        toFarRocketRight = Drivetrain.loadTrajectoryFromCSV("far_rocket_p1_right.csv");

        farRocketToLoadingRevLeft = Drivetrain.loadTrajectoryFromCSV("far_rocket_loading_reverse_left.csv");
        farRocketToLoadingRevRight = Drivetrain.loadTrajectoryFromCSV("far_rocket_loading_reverse_right.csv");
    }


}
