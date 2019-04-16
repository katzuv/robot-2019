package robot.utilities;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class MotorIssueDetector {
    private PowerDistributionPanel pdp;
    private double[] amperSeconds = new double[16];

    public MotorIssueDetector(PowerDistributionPanel pdp) {
        this.pdp = pdp;
    }

    private void updateAmpHours() {
        for (int i = 0; i < 16; i++) {
            amperSeconds[i] += 0.02 * pdp.getCurrent(i); //multiply current by cycle time
        }
    }

    public void update() {
        for (int[] group : Constants.pdpGroups) {
            boolean areAllOn = true;
            boolean areAllOff = true;
            for (int port : group) {
                if (pdp.getCurrent(port) > 1.e-4)
                    areAllOff = false;
                else
                    areAllOn = false;
            }
            if (!areAllOff && !areAllOn)
                DriverStation.reportError("There is a non working motor in motor group {" + group[0] + "," + group[1] + "," + group[2] + "}", false);
        }

        updateAmpHours();
        for (int i = 0; i < 16; i++) {
            if (amperSeconds[i] > Constants.ALLOWED_AMPER_SECONDS[i]) {
                DriverStation.reportWarning("Motor " + i + " has been taking a lot of amps, it should cool down", false);
            }
        }

    }

    public void reset() {
        amperSeconds = new double[16];
    }
}

class Constants {
    static int[][] pdpGroups = {
            {0, 1, 2},
            {15, 14, 13},
            {7, 12, 12}
    };

    static int[] ALLOWED_AMPER_SECONDS = {6000, 6000, 6000, 4200, 4200, 4200, 4200, 4000, 4200, 3000, 4200, 4200, 4000, 6000, 6000, 6000};
}