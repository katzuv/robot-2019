package robot.subsystems.drivetrain.talon_profiling;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;

import com.team254.lib.physics.DifferentialDrive;
import robot.subsystems.drivetrain.Drivetrain;

public class Profiles {

    public static BufferedTrajectoryPointStream toNearRocketLeft;
    public static BufferedTrajectoryPointStream toNearRocketRight;

    public static BufferedTrajectoryPointStream toLoadingStationNearRocketLeft;
    public static BufferedTrajectoryPointStream toLoadingStationNearRocketRight;

    public static BufferedTrajectoryPointStream loadingStationToFarRocketLeft;
    public static BufferedTrajectoryPointStream loadingStationToFarRocketRight;

    public static BufferedTrajectoryPointStream loadingStationToNearRocketLeft;
    public static BufferedTrajectoryPointStream loadingStationToNearRocketRight;

    static {
        toNearRocketLeft = Drivetrain.loadTrajectoryFromCSV("near_rocket_left.csv");
        toNearRocketRight = Drivetrain.loadTrajectoryFromCSV("near_rocket_right.csv");

        toLoadingStationNearRocketLeft = Drivetrain.loadTrajectoryFromCSV("near_rocket_loading_left.csv");
        toLoadingStationNearRocketRight = Drivetrain.loadTrajectoryFromCSV("near_rocket_loading_right.csv");

        loadingStationToFarRocketLeft = Drivetrain.loadTrajectoryFromCSV("loading_far_rocket_left.csv");
        loadingStationToFarRocketRight = Drivetrain.loadTrajectoryFromCSV("loading_far_rocket_right.csv");


        loadingStationToNearRocketLeft = Drivetrain.loadTrajectoryFromCSV("loading_to_near_left.csv");
        loadingStationToNearRocketRight = Drivetrain.loadTrajectoryFromCSV("loading_to_near_right.csv");

    }


}
