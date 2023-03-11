import java.util.AbstractList;
import java.util.ArrayList;
import java.util.List;

public class CSWandViaWPerLayer {
    private List<ChannelSegmentWrapper> csw;
    private List<ViaWrapper> viaw;

    CSWandViaWPerLayer() {
        csw = new ArrayList<ChannelSegmentWrapper>();
        viaw = new ArrayList<ViaWrapper>();
    }

    public void add_csw(ChannelSegmentWrapper cw) {
        csw.add(cw);
    }

    public void add_viaw(ViaWrapper vw) {
        viaw.add(vw);
    }

    public List<ChannelSegmentWrapper> get_csw() {
        return csw;
    }

    public List<ViaWrapper> get_viaw() {
        return viaw;
    }

    public int get_csw_size() {
        return csw.size();
    }

    public int get_viaw_size() {
        return viaw.size();
    }
}
