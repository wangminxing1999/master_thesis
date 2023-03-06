import chip.Chip;
import chip.pin.Pin;
import dev.yushen.wrapperapi.gurobi.solver.Model;
import gurobi.GRBException;

public class ConnectionChannelSolver {
    private Chip chip;
    private CSWandViaWPerLayer[] layers_content_record;
    private Model model;
    private int NumOfSegments;
    private int margin;

    ConnectionChannelSolver(Chip chip, int NumOfSegments, int margin) throws GRBException {
        this.chip = chip;
        this.NumOfSegments = NumOfSegments;
        this.margin = margin;
        this.layers_content_record = new CSWandViaWPerLayer[chip.getLayers().size()];
        this.model = new Model("My model");
    }

    public void add_single_connection_channel_constraint() {
        for(int i = 0; i < chip.getLayers().size(); i++) {
            for(int j = 0; j < chip.getLayers().get(i).getConnections().size(); j++) {
                Pin pin1 = chip.getLayers().get(i).getConnections().get(j).getPin1();
                Pin pin2 = chip.getLayers().get(i).getConnections().get(j).getPin2();
            }
        }
    }
}
