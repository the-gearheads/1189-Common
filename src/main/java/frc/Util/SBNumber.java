package frc.Util;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class SBNumber extends SBEntry{

    public SBNumber(SimpleWidget widget){
        super(widget);
    }

    public void setValue(double val){
        getWidget().getEntry().setDouble(val);
    }

    public double getValue(double defaultVal){
        return getWidget().getEntry().getDouble(defaultVal);
    }
}
