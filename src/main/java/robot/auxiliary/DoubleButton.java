package robot.auxiliary;


import edu.wpi.first.wpilibj.buttons.Button;

public class DoubleButton extends Button {

    private final Button button2;
    private final Button button1;
    private final boolean includeSecond;

    public DoubleButton(Button button1, Button button2, boolean includeSecond) {
        this.button1 = button1;
        this.button2 = button2;
        this.includeSecond = includeSecond;
    }

    public boolean get() {
        if (includeSecond) {
            return button1.get() && button2.get();
        }
        return button1.get() && !button2.get();
    }

}
