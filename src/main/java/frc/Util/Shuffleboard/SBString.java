package frc.Util.Shuffleboard;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class SBString extends SBEntry<SBString, String> {

    public SBString(SimpleWidget widget, String defaultVal){
        super(widget, defaultVal);
    }

    public void setValue(String val){
        getWidget().getEntry().setString(val);
    }

    public String getValue(String defaultVal){
        return getWidget().getEntry().getString(defaultVal);
    }
}
