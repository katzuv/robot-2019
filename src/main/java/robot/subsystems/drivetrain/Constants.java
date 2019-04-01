package robot.subsystems.drivetrain;

import com.team254.lib.physics.DCMotorTransmission;
import com.team254.lib.physics.DifferentialDrive;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d;
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.CentripetalAccelerationConstraint;
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.TimingConstraint;
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.VelocityLimitRegionConstraint;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.AccelerationKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;

import java.util.ArrayList;
import java.util.List;

import static robot.Robot.isRobotA;

public class Constants {
    public final static double ROBOT_WIDTH = 0.74; //the distance between the left and right wheels on the robot

    public static final boolean LEFT_ENCODER_REVERSED = isRobotA ? false : false;
    public static final boolean RIGHT_ENCODER_REVERSED = isRobotA ? false : false;

    public static final double LEFT_TICKS_PER_METER = isRobotA ? 2138.7750882690398 : 2138.7750882690398; // measured average distance (robot a), [1m / (diameter=0.1524 * pi)] * (ticks_per_meter=1024s) (robot b)
    public static final double RIGHT_TICKS_PER_METER = isRobotA ? 2138.7750882690398 : 2138.7750882690398;

    public static final int TALON_RUNNING_TIMEOUT_MS = isRobotA ? 0 : 0;
    public static final int TALON_TIMEOUT_MS = isRobotA ? 10 : 10;

    public static final double distanceFromEnd = 2;
    public static final double ENDING_TOLERANCE = 0.05;
    public static final double MIN_AIM = 0.06;

    public static final double MOTION_CRUISE_VELOCITY = 2; //Motion acceleration in M/S
    public static final double MOTION_ACCELERATION = 2; //Motion acceleration in M/S^2

    public static final double RAMSETE_PEAK_VELOCITY = 2.5;
    public static final double RAMSETE_PEAK_ACCELERATION = 2.5;
    public static final double VISION_VELOCITY = -0.75;
    public static final double SLOW_JOYSTICK_SPEED = 0.9; //multiplied by joystick value, keep at 1 for no changes.
    public static final double[] PIDFLeft = isRobotA ? new double[]{1.5, 0, 10, 1.3} : new double[]{1.5, 0, 10, 1.3};
    public static final double[] PIDFRight = isRobotA ? new double[]{2, 0.007, 20, 1.45} : new double[]{1.5, 0, 10, 1.3};
    public static final double kZeta = 0.8;
    public static final double kBeta = isRobotA ? 2 : 2;
    public static final double angleKp = isRobotA ? 1.6 : 1.37;

    //Model constants
    public static final double kRobotMass = 55;
    public static final double kRobotAngularDrag = 12;
    public static final Length kDriveWheelRadius = LengthKt.getInch(3);
    public static final Length kDriveTrackWidth = LengthKt.getInch(26);
    public static final double kRobotMomentOfIntertia = 10;

    public static final List<TimingConstraint<Pose2dWithCurvature>> constraints = new ArrayList<>();
    // (8 inches), the encoder sends 226
    // pulses every 360 degree turn
    static final boolean LEFT_MASTER_REVERSED = isRobotA ? false : false;
    static final boolean LEFT_SLAVE1_REVERSED = isRobotA ? false : false;
    static final boolean LEFT_SLAVE2_REVERSED = isRobotA ? false : false;
    static final boolean RIGHT_MASTER_REVERSED = isRobotA ? true : true;
    static final boolean RIGHT_SLAVE1_REVERSED = isRobotA ? true : true;
    static final boolean RIGHT_SLAVE2_REVERSED = isRobotA ? true : true;
    private static final double ACCELERATION_CONSTRAINT = 1.2;
    private static final double VELOCITY_CONSTRAINT = 3;
    private static final double RECTANGLE_1 = 4;
    private static final double RECTANGLE_2 = 7;
    private static final double RECTANGLE_3 = 8;
    private static final double RECTANGLE_4 = 20;
    public static double[] PIDVisionTurn = isRobotA ? new double[]{0.02, 0.0005, 0} : new double[]{0.02, 0.0006, 0.1};
    public static double pathAngleKp = isRobotA ? 2.3 : 2;

    public static double kDriveLeftKv = 0.7256;
    public static double kDriveLeftKa = 0.2766;
    public static double kDriveLeftKs = 0.8616;
    public static double kDriveRightKv = 0.7340;
    public static double kDriveRightKa = 0.2865;
    public static double kDriveRightKs = 0.8486;

    public static DCMotorTransmission leftTransmission = new DCMotorTransmission(
            1 / kDriveLeftKv,
            Math.pow(kDriveWheelRadius.getValue(), 2) * kRobotMass / (2.0 * kDriveLeftKa),
            kDriveLeftKs
    );
    public static DCMotorTransmission rightTransmission = new DCMotorTransmission(
            1 / kDriveRightKv,
            Math.pow(kDriveWheelRadius.getValue(), 2) * kRobotMass / (2.0 * kDriveRightKa),
            kDriveRightKs
    );
    public static final DifferentialDrive driveModel = new DifferentialDrive(
            kRobotMass,
            kRobotMomentOfIntertia,
            kRobotAngularDrag,
            kDriveWheelRadius.getValue(),
            kDriveTrackWidth.getValue() / 2,
            leftTransmission,
            rightTransmission
    );

    static {
        constraints.add(new CentripetalAccelerationConstraint(AccelerationKt.getAcceleration(LengthKt.getMeter(ACCELERATION_CONSTRAINT))));
        constraints.add(new VelocityLimitRegionConstraint(new Rectangle2d(LengthKt.getFeet(RECTANGLE_1), LengthKt.getFeet(RECTANGLE_2), LengthKt.getFeet(RECTANGLE_3), LengthKt.getFeet(RECTANGLE_4)), VelocityKt.getVelocity(LengthKt.getFeet(VELOCITY_CONSTRAINT))));
    }
}
