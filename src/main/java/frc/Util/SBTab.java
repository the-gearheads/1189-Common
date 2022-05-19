package frc.Util;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class SBTab { // SB stands for ShuffleBoard
    private ShuffleboardTab tab;
    private HashMap<String, SBEntry> widgets = new HashMap<String, SBEntry>();
    private HashMap<String, SBGroup> groups = new HashMap<String, SBGroup>();

    public SBTab(String name){
        this.tab = Shuffleboard.getTab(name);
        widgets = new HashMap<String,SBEntry>();
    }

    public SBGroup getGroup(String name, BuiltInLayouts view){
        if(groups.containsKey(name)){
            SBGroup group = groups.get(name);
            return groups.get(name);
        }
        SBGroup newGroup = new SBGroup(tab.getLayout(name, view));
        groups.put(name, newGroup);
        return newGroup;
    }

    public SBNumber getNumber(String name, double defaultVal){ // Set OR CREATES double
        if(widgets.containsKey(name)){
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

    public void periodic(){
        for(SBEntry widget : widgets.values()){
            widget.periodic();
        }
    }
}
