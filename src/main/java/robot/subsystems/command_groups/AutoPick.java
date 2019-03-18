package robot.subsystems.command_groups;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class AutoPick extends InstantCommand {

    boolean isHatch;

    public AutoPick(boolean isHatch){
        this.isHatch = isHatch;
    }

    protected void initialize(){
        if (isHatch){
            AutoPickUpHatch hatchPick = new AutoPickUpHatch();
            hatchPick.start();
        }else {
            AutoPickUpCargo cargoPick = new AutoPickUpCargo();
            cargoPick.start();
        }
    }

    @Override
    protected boolean isFinished(){
        return false;
    }
}
