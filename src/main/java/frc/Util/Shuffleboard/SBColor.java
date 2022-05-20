package frc.Util.Shuffleboard;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class SBColor extends SBEntry<SBColor, String>{

    public SBColor(SimpleWidget widget, String defaultVal){
        super(widget, defaultVal);
        getWidget().getEntry().setBoolean(true);
    }

    public void setValue(String color){
        this.setProperty("Color when true", color);
    }

    public String getValue(String defaultVal){ // Doesn't really do anything with defaultVal
        return (String) getProperties().get("Color when true");
    }
}
