package frc.utilwhatev.Shuffleboard;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class SBNumber extends SBEntry<SBNumber, Double> { 

    public SBNumber(SimpleWidget widget, Double defaultVal){
        super(widget, defaultVal, BuiltInWidgets.kTextView);
    }

    public void setValue(Double val){
        getWidget().getEntry().setDouble(val);
    }

    public Double getValue(Double defaultVal){
        return getWidget().getEntry().getDouble(defaultVal);
    }
}
