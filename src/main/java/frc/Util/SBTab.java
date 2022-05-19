package frc.Util;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class SBTab { // SB stands for ShuffleBoard
    private ShuffleboardTab tab;
    private HashMap<String, SBEntry> widgets = new HashMap<String, SBEntry>();

    public SBTab(String name){
        this.tab = Shuffleboard.getTab(name);
        widgets = new HashMap<String,SBEntry>();
    }

    public SBNumber getNumber(String name, double defaultVal){ // Set OR CREATES double
        if(widgets.containsKey(name)){
            tab.add(name, defaultVal);
            SBEntry entry = widgets.get(name);
            if(entry instanceof SBNumber){
                return (SBNumber) widgets.get(name);
            }else{
                DriverStation.reportError("widget was requested with incorrect type", true);
            }
        }
        SBNumber newEntry = new SBNumber(tab.add(name, defaultVal));
        widgets.put(name, newEntry);
        return newEntry;
    }

    public SBString getString(String name, String defaultVal){ // Set OR CREATES double
        if(widgets.containsKey(name)){
            SBEntry entry = widgets.get(name);
            if(entry instanceof SBString){
                return (SBString) widgets.get(name);
            }else{
                DriverStation.reportError("widget was requested with incorrect type", true);
            }
        }
        SBString newEntry = new SBString(tab.add(name, defaultVal));
        widgets.put(name, newEntry);
        return newEntry;
    }
    
    public SBBoolean getBoolean(String name, boolean defaultVal){ // Set OR CREATES double
        if(widgets.containsKey(name)){
            SBEntry entry = widgets.get(name);
            if(entry instanceof SBBoolean){
                return (SBBoolean) widgets.get(name);
            }else{
                DriverStation.reportError("widget was requested with incorrect type", true);
            }
        }
        SBBoolean newEntry = new SBBoolean(tab.add(name, defaultVal));
        widgets.put(name, newEntry);
        return (SBBoolean) newEntry;
    }
}
