package frc.util.Shuffleboard;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class SBTab { // SB stands for ShuffleBoard
    private ShuffleboardTab tab;
    private HashMap<String, SBEntry> widgets = new HashMap<String, SBEntry>();
    private HashMap<String, SBGroup> groups = new HashMap<String, SBGroup>();
    public static int colNum = 48;
    public static int rowNum = 22;

    public SBTab(String name){
        this.tab = Shuffleboard.getTab(name);
        widgets = new HashMap<String,SBEntry>();
    }

    public ShuffleboardTab getTab(){
        return tab;
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
        SBNumber newEntry = new SBNumber(tab.add(name, defaultVal), defaultVal);
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
        SBString newEntry = new SBString(tab.add(name, defaultVal), defaultVal);
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
        SBBoolean newEntry = new SBBoolean(tab.add(name, defaultVal), defaultVal);
        widgets.put(name, newEntry);
        return (SBBoolean) newEntry;
    }

    public SBColor getColor(String name, String defaultVal){ // Set OR CREATES double
        if(widgets.containsKey(name)){
            SBEntry entry = widgets.get(name);
            if(entry instanceof SBBoolean){
                return (SBColor) widgets.get(name);
            }else{
                DriverStation.reportError("widget was requested with incorrect type", true);
            }
        }
        SBColor newEntry = new SBColor(tab.add(name, defaultVal), defaultVal);
        widgets.put(name, newEntry);
        return (SBColor) newEntry;
    }
    
    public SBGroup getGroup(String name){
        if(groups.containsKey(name)){
            SBGroup group = groups.get(name);
            return group;
        }
        SBGroup newGroup = new SBGroup();
        groups.put(name, newGroup);
        return newGroup;
    }

    public SBNumberGroup getNumberGroup(String name, double defaultVal){ // Set OR CREATES double
        if(groups.containsKey(name)){
            SBGroup group = groups.get(name);
            if(group instanceof SBNumberGroup){
                return (SBNumberGroup) groups.get(name);
            }else{
                DriverStation.reportError("group was requested with incorrect type", true);
            }
        }

        SBNumber graph = this.getNumber(name + " graph", defaultVal);
        graph.setView(BuiltInWidgets.kGraph);
        SBNumber text  = this.getNumber(name, defaultVal);
        SBNumberGroup newNumberGroup = new SBNumberGroup(graph, text);
        groups.put(name, newNumberGroup);
        return (SBNumberGroup) newNumberGroup;
    }

    public void periodic(){
        for(SBEntry widget : widgets.values()){
            widget.periodic();
        }
    }
}
