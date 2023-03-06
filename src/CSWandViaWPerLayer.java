import java.util.AbstractList;
import java.util.ArrayList;
import java.util.List;

public class CSWandViaWPerLayer {
    private List<ConnectionWrapper> csw;
    private List<ViaWrapper> viaw;

    CSWandViaWPerLayer() {
        csw = new ArrayList<ConnectionWrapper>();
        viaw = new ArrayList<ViaWrapper>();
    }

    public void add_csw(ConnectionWrapper cw) {
        csw.add(cw);
    }

    public void add_viaw(ViaWrapper vw) {
        viaw.add(vw);
    }

    public ConnectionWrapper get_csw(int id) {
        return csw.get(id);
    }

    public ViaWrapper get_viaw(int id) {
        return viaw.get(id);
    }

    public int get_csw_size() {
        return csw.size();
    }

    public int get_viaw_size() {
        return viaw.size();
    }
}
