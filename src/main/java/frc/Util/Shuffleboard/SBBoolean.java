package frc.util.Shuffleboard;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class SBBoolean extends SBEntry<SBBoolean, Boolean>{

    public SBBoolean(SimpleWidget widget, Boolean defaultVal){
        super(widget, defaultVal);
    }

    public void setValue(Boolean val){
        getWidget().getEntry().setBoolean(val);
    }

    public Boolean getValue(Boolean defaultVal){
        return getWidget().getEntry().getBoolean(defaultVal);
    }
}
