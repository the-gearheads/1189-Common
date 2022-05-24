package frc.utilwhatev.Shuffleboard;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;
import static java.util.Map.entry;    


import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public abstract class SBEntry<T extends SBEntry<T, R>, R> {
    private SimpleWidget widget;
    private Map<String, Object> properties;
    private Function<R, R> lambda = (current)->{return current;};
    private R defaultVal;
    private WidgetSize size = new WidgetSize(3,3);
    private BuiltInWidgets view;
    private Map<BuiltInWidgets, WidgetSize> minSizes = Map.ofEntries(
        entry(BuiltInWidgets.kTextView, new WidgetSize(3,3)),
        entry(BuiltInWidgets.kBooleanBox, new WidgetSize(3, 3)),
        entry(BuiltInWidgets.kToggleButton, new WidgetSize(3, 3)),
        entry(BuiltInWidgets.kGraph, new WidgetSize(10, 10))
    );

    SBEntry(SimpleWidget widget, R defaultVal, BuiltInWidgets view){
        this.widget = widget;
        this.defaultVal = defaultVal;
        setView(view);
    }

    public SimpleWidget getWidget(){
        return this.widget;
    }

    public T setSize(int width, int height){
        width = Math.max(width, minSizes.get(view).width);
        height = Math.max(height, minSizes.get(view).height);

        this.widget.withSize(width,height);
        size.width = width;
        size.height = height;

        return (T) this;
    }

    public T setPosition(int col, int row){
        this.widget.withPosition(col,row);

        return (T) this;
    }

    public T setProperties(Map<String, Object> properties){
        this.properties = properties;
        this.widget.withProperties(properties);

        return (T) this;
    }

    public T setProperty(String name, Object property){
        if(properties.containsKey(name)){
            properties.remove(name);
        }
        properties.put(name, property);

        setProperties(properties);

        return (T) this;
    }

    public Map<String, Object> getProperties(){
        return this.properties;
    }

    public T setView(BuiltInWidgets view){
        this.view = view;
        this.widget.withWidget(view);
        setSize(this.size.width, this.size.height);
        
        return (T) this;
    }

    public T setPeriodic(Function<R,R> lambda){
        this.lambda = lambda;

        return (T) this;   
    }

    public T setPeriodic(Consumer<R> lambda){
        this.lambda = (current) -> {
            lambda.accept(current);
            return current;
        };

        return (T) this;   
    }

    public T setPeriodic(Supplier<R> lambda){
        this.lambda = (current) -> {
            return lambda.get();
        };

        return (T) this;   
    }

    public void periodic(){
        this.setValue(lambda.apply(this.getValue(defaultVal)));
    }

    public WidgetSize getSize(){
        return size;
    }

    public abstract void setValue(R value);

    public abstract R getValue(R defaultVal);
}