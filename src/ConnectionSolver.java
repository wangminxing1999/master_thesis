import chip.Chip;
import chip.ChipRequest;
import chip.connection.ChannelSegment;
import chip.connection.Connection;
import dev.yushen.wrapperapi.gurobi.solver.Model;
import dev.yushen.wrapperapi.gurobi.variable.BinaryVariable;
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
        for(int i = 0; i < this.connection_wrapper.length; i++) {
            this.connection_wrapper[i] = new ConnectionWrapper(this.chip_request.getConnections().get(i), NumOfSegments);
            for(int j = 0; j < this.connection_wrapper[i].channel_segment.length; j++){
                IntegerVariable x1 = model.intVar("x1", 0, chip.getWidth());
                IntegerVariable y1 = model.intVar("y1", 0, chip.getHeight());
                IntegerVariable x2 = model.intVar("x2", 0, chip.getWidth());
                IntegerVariable y2 = model.intVar("y2", 0, chip.getHeight());
                if(j < this.connection_wrapper[i].channel_segment.length/2)
                    this.connection_wrapper[i].channel_segment[j] = new solve_ChannelSegment(x1, y1, x2, y2, this.connection_wrapper[i].getConnection(),this.connection_wrapper[i].getConnection().getPin1().getModule().getLayer());
                else {
                    this.connection_wrapper[i].channel_segment[j] = new solve_ChannelSegment(x1, y1, x2, y2, this.connection_wrapper[i].getConnection(),this.connection_wrapper[i].getConnection().getPin2().getModule().getLayer());
                }
                IntegerVariable x = model.intVar("x",0,chip.getWidth());
                IntegerVariable y = model.intVar("y",0,chip.getHeight());
                this.connection_wrapper[i].via = new solve_Via(this.connection_wrapper[i].getConnection().getPin1().getModule().getLayer(),this.connection_wrapper[i].getConnection().getPin2().getModule().getLayer(), x, y);
            }
        }
    }
    public void add_channelsegment_self_constraint() throws GRBException {
        for(int i = 0; i < this.connection_wrapper.length; i++) {
            this.model.post(this.connection_wrapper[i].channel_segment[0].getX1(), '=', this.connection_wrapper[i].getConnection().getPin1().getX());
            this.model.post(this.connection_wrapper[i].channel_segment[0].getY1(), '=', this.connection_wrapper[i].getConnection().getPin1().getY());
            this.model.post(this.connection_wrapper[i].channel_segment[this.NumOfSegments-1].getX2(), '=', this.connection_wrapper[i].getConnection().getPin2().getX());
            this.model.post(this.connection_wrapper[i].channel_segment[this.NumOfSegments-1].getY2(), '=', this.connection_wrapper[i].getConnection().getPin2().getY());
            for(int j = 0; j < this.connection_wrapper[i].channel_segment.length; j++) {
                BinaryVariable temp1 = this.model.binVar("temp1");
                BinaryVariable temp2 = this.model.binVar("temp2");
                this.model.post(temp1.add(temp2), '=', 1);
                this.model.post(this.connection_wrapper[i].channel_segment[j].getX1(), '<', this.connection_wrapper[i].channel_segment[j].getX2().add(temp1.mul(this.model.M())));
                this.model.post(this.connection_wrapper[i].channel_segment[j].getX2(), '<', this.connection_wrapper[i].channel_segment[j].getX1().add(temp1.mul(this.model.M())));
                this.model.post(this.connection_wrapper[i].channel_segment[j].getY1(), '<', this.connection_wrapper[i].channel_segment[j].getY2().add(temp2.mul(this.model.M())));
                this.model.post(this.connection_wrapper[i].channel_segment[j].getY2(), '<', this.connection_wrapper[i].channel_segment[j].getY1().add(temp2.mul(this.model.M())));
                if( j <= this.connection_wrapper[i].channel_segment.length - 2) {
                    this.model.post(this.connection_wrapper[i].channel_segment[j].getX2(), '=', this.connection_wrapper[i].channel_segment[j+1].getX1());
                    this.model.post(this.connection_wrapper[i].channel_segment[j].getY2(), '=', this.connection_wrapper[i].channel_segment[j+1].getY1());
                }
            }
            this.model.post(this.connection_wrapper[i].via.getX(), '=', this.connection_wrapper[i].channel_segment[NumOfSegments/2].getX1());
            this.model.post(this.connection_wrapper[i].via.getY(), '=', this.connection_wrapper[i].channel_segment[NumOfSegments/2].getY1());
        }
    }

    public void add_avoid_collision_against_modules() {


    }
    public void solve_connection_wrapper() {

    }

    public Chip getChip() {return this.chip;}
    public ChipRequest getChip_request() {return this.chip_request;}
}
