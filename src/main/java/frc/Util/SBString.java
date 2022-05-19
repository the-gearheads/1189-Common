package frc.Util;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class SBString extends SBEntry{

    public SBString(SimpleWidget widget){
        super(widget);
    }

    public void setValue(String val){
        getWidget().getEntry().setString(val);
    }

    public String getValue(String defaultVal){
        return getWidget().getEntry().getString(defaultVal);
    }
}
