import chip.connection.ChannelSegment;
import chip.connection.Connection;
import chip.layer.Layer;
import dev.yushen.wrapperapi.gurobi.variable.IntegerVariable;
import gurobi.GRBException;

public class ChannelSegmentWrapper {
    public IntegerVariable x1,y1,x2,y2;
    public Connection conn;
    public Layer layer;
    private ChannelSegment ch_seg;

    ChannelSegmentWrapper(ChannelSegment ch_seg) {
        this.ch_seg = ch_seg;
    }

    public ChannelSegment getChannelSegment() {
        return ch_seg;
    }

    public void fill_parameter() throws GRBException {
        ch_seg.setX1(x1.getValue());
        ch_seg.setY1(y1.getValue());
        ch_seg.setX2(x2.getValue());
        ch_seg.setY2(y2.getValue());
        ch_seg.setConnection(conn);
        ch_seg.setLayer(layer);
    }
}
