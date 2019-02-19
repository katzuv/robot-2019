package robot.subsystems.commandGroups;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class AutoPick extends InstantCommand {

    boolean isHatch;

    public AutoPick(boolean isHatch){
        this.isHatch = isHatch;
    }

    protected void initialize(){
        if (isHatch){
            new AutoPickUpHatch();
        }else {
            new AutoPickUpCargo();
        }
    }

    @Override
    protected boolean isFinished(){
        return false;
    }
}
