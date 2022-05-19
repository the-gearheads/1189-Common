package frc.Util;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class SBBoolean extends SBEntry{

    public SBBoolean(SimpleWidget widget){
        super(widget);
    }

    public void setValue(boolean val){
        getWidget().getEntry().setBoolean(val);
    }

    public boolean getBoolean(boolean defaultVal){
        return getWidget().getEntry().getBoolean(defaultVal);
    }
}
