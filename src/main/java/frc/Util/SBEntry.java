package frc.Util;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class SBEntry {
    private SimpleWidget widget;
    private Map<String, Object> properties;

    SBEntry(SimpleWidget widget){
        this.widget = widget;
    }

    public SimpleWidget getWidget(){
        return this.widget;
    }

    public void setSize(int width, int height){
        this.widget.withSize(width,height);
    }

    public void setPosition(int col, int row){
        this.widget.withPosition(col,row);
    }

    public void setProperties(Map<String, Object> properties){
        this.properties = properties;
        this.widget.withProperties(properties);
    }

    public void setProperty(String name, Object property){
        if(properties.containsKey(name)){
            properties.remove(name);
        }
        properties.put(name, property);

        setProperties(properties);
    }

    public void setView(BuiltInWidgets view){
        this.widget.withWidget(view);
    }
}