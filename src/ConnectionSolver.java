import chip.Chip;
import chip.ChipRequest;
import chip.connection.ChannelSegment;
import chip.connection.Connection;
import dev.yushen.wrapperapi.gurobi.solver.Model;
import dev.yushen.wrapperapi.gurobi.variable.IntegerVariable;
import gurobi.GRBException;

public class ConnectionSolver {
    private Chip chip;
    private ChipRequest chip_request;
    public ConnectionWrapper[] connection_wrapper;
    public Model model;
    public int NumOfSegments;

    public ConnectionSolver(Chip chip, ChipRequest chip_request, int NumOfSegments) throws GRBException {
        this.model = new Model("chip_channel_construction");
        this.chip = chip;
        this.chip_request = chip_request;
        this.connection_wrapper = new ConnectionWrapper[chip_request.getConnections().size()];
        this.NumOfSegments = NumOfSegments;
    }

    public void solve_connection_wrapper() throws GRBException {
        for(int i = 0; i < this.connection_wrapper.length; i++) {
            this.connection_wrapper[i] = new ConnectionWrapper(this.chip_request.getConnections().get(i), NumOfSegments);
            if(this.connection_wrapper[i].getConnection().getPin1().getModule().getLayer() == this.connection_wrapper[i].getConnection().getPin2().getModule().getLayer()) {
                for(solve_ChannelSegment channel_segment : this.connection_wrapper[i].channel_segment) {
                    IntegerVariable x1 = model.intVar("x1", 0, chip.getWidth());
                    IntegerVariable y1 = model.intVar("y1", 0, chip.getHeight());
                    IntegerVariable x2 = model.intVar("x2", 0, chip.getWidth());
                    IntegerVariable y2 = model.intVar("y2", 0, chip.getHeight());
                    channel_segment = new solve_ChannelSegment(x1, y1, x2, y2, this.connection_wrapper[i].getConnection(),this.connection_wrapper[i].getConnection().getPin1().getModule().getLayer());
                }
            }
        }
    }

    public Chip getChip() {return this.chip;}
    public ChipRequest getChip_request() {return this.chip_request;}
}
