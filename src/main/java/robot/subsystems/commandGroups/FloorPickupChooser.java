package robot.subsystems.commandGroups;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class FloorPickupChooser extends InstantCommand {

    boolean isHatch;

    public FloorPickupChooser(boolean isHatch){
        this.isHatch = isHatch;
    }

    protected void initialize(){
        if (isHatch){
            FloorPickupHatch hatchPick = new FloorPickupHatch();
            hatchPick.start();
        }else {
            FloorPickupCargo cargoPick = new FloorPickupCargo();
            cargoPick.start();
        }
    }

    @Override
    protected boolean isFinished(){
        return false;
    }
}
