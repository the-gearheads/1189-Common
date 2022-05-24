package frc.utilwhatev.Shuffleboard;

import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

public class SBNumberGroup extends SBGroup{

    private SBNumber graph;
    private SBNumber text;

    public SBNumberGroup(SBNumber graph, SBNumber text){
        super();

        this.graph = graph;
        this.text = text;
        this.append(graph);
        this.append(text);
    }

    public void setPeriodic(Function<Double,Double> lambda){
        graph.setPeriodic(lambda);
        text.setPeriodic(lambda);
    }

    public void setPeriodic(Consumer<Double> lambda){
        graph.setPeriodic(lambda);
        text.setPeriodic(lambda);
    }

    public void setPeriodic(Supplier<Double> lambda){
        graph.setPeriodic(lambda);
        text.setPeriodic(lambda);
    }
}
