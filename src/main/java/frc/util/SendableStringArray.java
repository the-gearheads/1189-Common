package frc.util;

import java.util.function.Supplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SendableStringArray implements Sendable{
    private Supplier<String> first;
    private Supplier<String> second;
    private Supplier<String> third;

    public SendableStringArray(Supplier<String> first, Supplier<String> second, Supplier<String> third){
        this.first=first;
        this.second=second;
        this.third=third;
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("String Array");
      builder.addStringProperty("1", first, (x)->{});
      builder.addStringProperty("2", second, (x)->{});
      builder.addStringProperty("3", third, (x)->{});

      
    }
}
