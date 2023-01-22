import chip.connection.Connection;
import chip.layer.Layer;
import dev.yushen.wrapperapi.gurobi.solver.Model;
import dev.yushen.wrapperapi.gurobi.variable.IntegerVariable;
import gurobi.GRBException;

public class solve_ChannelSegment {
    private IntegerVariable x1;
    private IntegerVariable x2;
    private IntegerVariable y1;
    private IntegerVariable y2;
    private Connection connection;
    private Layer layer;

    public solve_ChannelSegment(IntegerVariable x1, IntegerVariable y1, IntegerVariable x2, IntegerVariable y2, Connection connection, Layer layer) {
        this.x1 = x1;
        this.x2 = x2;
        this.y1 = y1;
        this.y2 = y2;
        this.connection = connection;
        this.layer = layer;
    }
    public Layer getLayer() {
        return this.layer;
    }

    public void setLayer(Layer layer) {
        this.layer = layer;
    }

    public IntegerVariable getX1() {
        return this.x1;
    }

    public void setX1(IntegerVariable x1) {
        this.x1 = x1;
    }

    public IntegerVariable getX2() {
        return this.x2;
    }

    public void setX2(IntegerVariable x2) {
        this.x2 = x2;
    }

    public IntegerVariable getY1() {
        return this.y1;
    }

    public void setY1(IntegerVariable y1) {
        this.y1 = y1;
    }

    public IntegerVariable getY2() {
        return this.y2;
    }

    public void setY2(IntegerVariable y2) {
        this.y2 = y2;
    }

    public Connection getConnection() {
        return this.connection;
    }

    public void setConnection(Connection connection) {
        this.connection = connection;
    }
}
