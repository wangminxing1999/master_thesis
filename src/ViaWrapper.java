import chip.connection.Via;
import chip.layer.Layer;
import dev.yushen.wrapperapi.gurobi.variable.IntegerVariable;
import gurobi.GRBException;

public class ViaWrapper {
    public IntegerVariable x,y;
    public Layer from,to;
    private Via via;

    public ViaWrapper(IntegerVariable x, IntegerVariable y, Layer from, Layer to) {
        this.x = x;
        this.y = y;
        this.from = from;
        this.to = to;
        via = new Via(from,to,0,0);
    }

    public Via getVia() {
        return via;
    }

    public void fill_parameter() throws GRBException {
        via.setX(x.getValue());
        via.setY(y.getValue());
        via.setFrom(from);
        via.setTo(to);
    }
}
