package frc.util.Shuffleboard;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class SBGroup { // NOT CURRENTLY USED BECAUSE SB LAYOUTS SUCCKKKKKKKKKK
                       // EDIT: WORKS, SINCE DOESN'T USE LAYOUTS ANYMORE

    private ArrayList<SBEntry> entries;
    private int width;
    private int col;
    private int row;

    public SBGroup(){
        this.entries = new ArrayList<SBEntry>();
    }

    public SBGroup setWidth(int width){
        this.width = width;
        updateWidth();

        return this;
    }

    private void updateWidth(){
        for(SBEntry entry: entries){
            entry.setSize(this.width, entry.getSize().height);
        }
    }

    public SBGroup setPosition(int col, int row){
        this.col = col;
        this.row = row;
        updatePosition();
        return this;
    }

    private void updatePosition(){
        int innerRow = 0;
        for(SBEntry entry: entries){
            entry.setPosition(this.col, this.row+innerRow);
            innerRow+=entry.getSize().height;
        }
    }

    public void append(SBEntry entry){
        entries.add(entry);
        if(entry.getSize().width > this.width){
            this.width = entry.getSize().width;
        }
        updateWidth();
        updatePosition();
    }
    public WidgetSize getSize(){
        int width = this.width;
        int height = 0;
        for(SBEntry entry: entries){
            height+=entry.getSize().height;
        }
        return new WidgetSize(width, height);
    }
}
