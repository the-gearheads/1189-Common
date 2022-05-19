package frc.Util.Shuffleboard;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class SBGroup { // NOT CURRENTLY USED BECAUSE SB LAYOUTS SUCCKKKKKKKKKK
    private ShuffleboardLayout layout;

    public SBGroup(ShuffleboardLayout layout){
        this.layout = layout;
    }

    public void setSize(int width, int height){
        this.layout.withSize(width,height);
    }

    public void setPosition(int col, int row){
        this.layout.withPosition(col,row);
    }
}
