package frc.util.Shuffleboard;

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

    public SBNumberGroup setPeriodic(Function<Double,Double> lambda){
        graph.setPeriodic(lambda);
        text.setPeriodic(lambda);

        return this;
    }

    public SBNumberGroup setPeriodic(Consumer<Double> lambda){
        graph.setPeriodic(lambda);
        text.setPeriodic(lambda);

        return this;
    }

    public SBNumberGroup setPeriodic(Supplier<Double> lambda){
        graph.setPeriodic(lambda);
        text.setPeriodic(lambda);

        return this;
    }

    @Override
    public SBNumberGroup setPosition(int col, int row){
        super.setPosition(col, row);
        return this;
    }

    @Override
    public SBNumberGroup setWidth(int width){
        super.setWidth(width);
        return this;
    }
}
