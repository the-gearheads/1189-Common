package frc.util.Shuffleboard;

import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public abstract class SBEntry<T extends SBEntry<T, R>, R> {
    private SimpleWidget widget;
    private Map<String, Object> properties;
    private Function<R, R> lambda = (current)->{return current;};
    private R defaultVal;
    private int width = 0;
    private int height = 0;

    SBEntry(SimpleWidget widget, R defaultVal){
        this.widget = widget;
        this.defaultVal = defaultVal;
        this.width = 3;
        this.height = 3;
    }

    public SimpleWidget getWidget(){
        return this.widget;
    }

    public T setSize(int width, int height){
        this.widget.withSize(width,height);
        this.width = width;
        this.height = height;
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
        this.widget.withWidget(view);
        
        if(view.equals(BuiltInWidgets.kGraph)){
            if(this.width < 10 || this.height < 10){
                this.width = 10;
                this.height = 10;
            }
        }
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

    public int getWidth(){
        return width;
    }

    public int getHeight(){
        return height;
    }

    public abstract void setValue(R value);

    public abstract R getValue(R defaultVal);
}