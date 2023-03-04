import chip.connection.Via;
import chip.layer.Layer;
import dev.yushen.wrapperapi.gurobi.variable.IntegerVariable;
import gurobi.GRBException;

public class ViaWrapper {
    public IntegerVariable x,y;
    public Layer from,to;
    private Via via;

    public ViaWrapper(Via via) {
        this.via = via;
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
