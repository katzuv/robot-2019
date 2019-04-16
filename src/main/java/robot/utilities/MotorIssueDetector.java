package robot.utilities;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    private int[] checkGroups() {
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
                return group;
        }
        return null;
    }

    public void update() {
        updateAmpHours();
        int[] problematic = checkGroups();
        double[] sendableList = new double[16];
        if (problematic != null) {

            for(int i = 0; i < 16; i++)
                sendableList[i] = problematic[i];
            SmartDashboard.putNumberArray("PROBLEMATIC MOTOR GROUP", sendableList);
        }
        for (int i = 0; i < 16; i++) {
            if (amperSeconds[i] > Constants.ALLOWED_AMPER_SECONDS[i]) {

            }
        }

    }

    public void reset() {

    }
}

class Constants {
    static int[][] pdpGroups = {
            {0, 1, 2},
            {15, 14, 13},
            {7, 12, 12}
    };

    static int[] ALLOWED_AMPER_SECONDS = {4200, 4200, 4200, 4200, 4200, 4200, 4200, 4200, 4200, 4200, 4200, 4200, 4200, 4200, 4200, 4200};
}