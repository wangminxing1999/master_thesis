import chip.Chip;
import chip.ChipRequest;
import chip.connection.ChannelSegment;
import chip.connection.Connection;
import chip.module.Module;
import dev.yushen.wrapperapi.gurobi.solver.Model;
import dev.yushen.wrapperapi.gurobi.variable.BinaryVariable;
import dev.yushen.wrapperapi.gurobi.variable.IntegerVariable;
import gurobi.GRBException;

import java.util.List;

public class ConnectionSolver {
    private Chip chip;
    private ChipRequest chip_request;
    public ConnectionWrapper[] connection_wrapper;
    public Model model;
    public int NumOfSegments;
    public Store_solve_channelsegments_layers[] layer_content_record;

    public int margin;
    public ConnectionSolver(Chip chip, ChipRequest chip_request, int NumOfSegments, int margin) throws GRBException {
        this.margin = margin;
        this.model = new Model("chip_channel_construction");
        this.chip = chip;
        this.chip_request = chip_request;
        this.connection_wrapper = new ConnectionWrapper[chip_request.getConnections().size()];
        this.NumOfSegments = NumOfSegments;
        this.layer_content_record = new Store_solve_channelsegments_layers[this.chip.getLayers().size()];
        for(int i = 0; i < this.chip.getLayers().size(); i++) {
            layer_content_record[i] = new Store_solve_channelsegments_layers();
        }
        for(int i = 0; i < this.connection_wrapper.length; i++) {
            this.connection_wrapper[i] = new ConnectionWrapper(this.chip_request.getConnections().get(i), NumOfSegments);
            for(int j = 0; j < this.connection_wrapper[i].channel_segment.length; j++){
                IntegerVariable x1 = model.intVar("x1", 0, chip.getWidth());
                IntegerVariable y1 = model.intVar("y1", 0, chip.getHeight());
                IntegerVariable x2 = model.intVar("x2", 0, chip.getWidth());
                IntegerVariable y2 = model.intVar("y2", 0, chip.getHeight());
                if(j < this.connection_wrapper[i].channel_segment.length/2){
                    int id = this.connection_wrapper[i].getConnection().getPin1().getModule().getLayer().getId();
                    this.connection_wrapper[i].channel_segment[j] = new solve_ChannelSegment(x1, y1, x2, y2, this.connection_wrapper[i].getConnection(),this.connection_wrapper[i].getConnection().getPin1().getModule().getLayer());
                    this.layer_content_record[id].store.add(this.connection_wrapper[i].channel_segment[j]);
                }
                else {
                    int id = this.connection_wrapper[i].getConnection().getPin2().getModule().getLayer().getId();
                    this.connection_wrapper[i].channel_segment[j] = new solve_ChannelSegment(x1, y1, x2, y2, this.connection_wrapper[i].getConnection(),this.connection_wrapper[i].getConnection().getPin2().getModule().getLayer());
                    this.layer_content_record[id].store.add(this.connection_wrapper[i].channel_segment[j]);
                }
                IntegerVariable x = model.intVar("x",0,chip.getWidth());
                IntegerVariable y = model.intVar("y",0,chip.getHeight());
                this.connection_wrapper[i].via = new solve_Via(this.connection_wrapper[i].getConnection().getPin1().getModule().getLayer(),this.connection_wrapper[i].getConnection().getPin2().getModule().getLayer(), x, y);
                int id1 = this.connection_wrapper[i].getConnection().getPin1().getModule().getLayer().getId();
                int id2 = this.connection_wrapper[i].getConnection().getPin2().getModule().getLayer().getId();
                if(id1 > id2) {
                    for(int k = id2 + 1; k <= id1 - 1; k++) {
                        this.layer_content_record[k].store_via.add(this.connection_wrapper[i].via);
                    }
                }
                if(id1 < id2) {
                    for(int k = id1 + 1; k <= id2 - 1; k++) {
                        this.layer_content_record[k].store_via.add(this.connection_wrapper[i].via);
                    }
                }
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

    public void add_avoid_collision_against_modules_or_each_other() throws GRBException {

        for(int i = 0; i < this.chip.getLayers().size(); i++) {

            List<Module> modules = this.chip.getLayers().get(i).getModules();
            List<solve_ChannelSegment> solve_channelSegments = this.layer_content_record[i].store;
            List<solve_Via> solve_vias = this.layer_content_record[i].store_via;

            for(int j = 0; j < modules.size(); j++) {
                int left = 0, right = 0, up = 0, down = 0;
                switch(modules.get(j).getOrientation()) {
                    case D0 :
                        left = modules.get(j).getX() - modules.get(j).getWidth()/2;
                        right = modules.get(j).getX() + modules.get(j).getWidth()/2;
                        down = modules.get(j).getY() - modules.get(j).getHeight()/2;
                        up = modules.get(j).getY() + modules.get(j).getHeight()/2;
                        break;
                    case D90 :
                        left = modules.get(j).getX() - modules.get(j).getHeight()/2;
                        right = modules.get(j).getX() + modules.get(j).getHeight()/2;
                        down = modules.get(j).getY() - modules.get(j).getWidth()/2;
                        up = modules.get(j).getY() + modules.get(j).getWidth()/2;
                        break;
                    case D180 :
                        left = modules.get(j).getX() - modules.get(j).getWidth()/2;
                        right = modules.get(j).getX() + modules.get(j).getWidth()/2;
                        down = modules.get(j).getY() - modules.get(j).getHeight()/2;
                        up = modules.get(j).getY() + modules.get(j).getHeight()/2;
                        break;
                    case D270 :
                        left = modules.get(j).getX() - modules.get(j).getHeight()/2;
                        right = modules.get(j).getX() + modules.get(j).getHeight()/2;
                        down = modules.get(j).getY() - modules.get(j).getWidth()/2;
                        up = modules.get(j).getY() + modules.get(j).getWidth()/2;
                        break;
                }

                for(int k = 0; k < solve_vias.size(); k++) {
                    BinaryVariable b1 = model.binVar("b1");
                    BinaryVariable b2 = model.binVar("b2");
                    BinaryVariable b3 = model.binVar("b3");
                    BinaryVariable b4 = model.binVar("b4");

                    model.post(b1.add(b2).add(b3).add(b4), '=', 3);

                    model.post(solve_vias.get(k).getX().add(this.margin).sub(b1.mul(model.M())), '<', left);
                    model.post(solve_vias.get(k).getX().sub(this.margin).add(b2.mul(model.M())), '>', right);
                    model.post(solve_vias.get(k).getY().add(this.margin).sub(b3.mul(model.M())), '<', down);
                    model.post(solve_vias.get(k).getY().sub(this.margin).add(b4.mul(model.M())), '>', up);
                }

                for(int k = 0; k < solve_channelSegments.size(); k++) {
                    BinaryVariable b1 = model.binVar("b1");
                    BinaryVariable b2 = model.binVar("b2");
                    BinaryVariable b3 = model.binVar("b3");
                    BinaryVariable b4 = model.binVar("b4");
                    BinaryVariable b5 = model.binVar("b5");
                    BinaryVariable b6 = model.binVar("b6");
                    BinaryVariable b7 = model.binVar("b7");
                    BinaryVariable b8 = model.binVar("b8");

                    model.post(b1.add(b2).add(b3).add(b4).add(b5).add(b6).add(b7).add(b8),'=',7);

                    model.post(solve_channelSegments.get(k).getY1(),'<',solve_channelSegments.get(k).getY2().add(b1.mul(model.M())));
                    model.post(solve_channelSegments.get(k).getY2(),'<',solve_channelSegments.get(k).getY1().add(b1.mul(model.M())));
                    model.post(solve_channelSegments.get(k).getY1().add(this.margin).sub(b1.mul(model.M())),'<',down);

                    model.post(solve_channelSegments.get(k).getY1(),'<',solve_channelSegments.get(k).getY2().add(b2.mul(model.M())));
                    model.post(solve_channelSegments.get(k).getY2(),'<',solve_channelSegments.get(k).getY1().add(b2.mul(model.M())));
                    model.post(solve_channelSegments.get(k).getY1().sub(this.margin).add(b2.mul(model.M())),'>',up);

                    model.post(solve_channelSegments.get(k).getY1(),'<',solve_channelSegments.get(k).getY2().add(b3.mul(model.M())));
                    model.post(solve_channelSegments.get(k).getY2(),'<',solve_channelSegments.get(k).getY1().add(b3.mul(model.M())));
                    model.post(solve_channelSegments.get(k).getY1().add(b3.mul(model.M())),'>',down);
                    model.post(solve_channelSegments.get(k).getY1().sub(b3.mul(model.M())),'<',up);
                    model.post(solve_channelSegments.get(k).getX1().sub(b3.mul(model.M())),'<',left);
                    model.post(solve_channelSegments.get(k).getX2().sub(b3.mul(model.M())),'<',left);

                    model.post(solve_channelSegments.get(k).getY1(),'<',solve_channelSegments.get(k).getY2().add(b4.mul(model.M())));
                    model.post(solve_channelSegments.get(k).getY2(),'<',solve_channelSegments.get(k).getY1().add(b4.mul(model.M())));
                    model.post(solve_channelSegments.get(k).getY1().add(b4.mul(model.M())),'>',down);
                    model.post(solve_channelSegments.get(k).getY1().sub(b4.mul(model.M())),'<',up);
                    model.post(solve_channelSegments.get(k).getX1().add(b4.mul(model.M())),'>',right);
                    model.post(solve_channelSegments.get(k).getX2().add(b4.mul(model.M())),'>',right);

                    model.post(solve_channelSegments.get(k).getX1(),'<',solve_channelSegments.get(k).getX2().add(b5.mul(model.M())));
                    model.post(solve_channelSegments.get(k).getX2(),'<',solve_channelSegments.get(k).getX1().add(b5.mul(model.M())));
                    model.post(solve_channelSegments.get(k).getX1().add(this.margin).sub(b5.mul(model.M())),'<',left);

                    model.post(solve_channelSegments.get(k).getX1(),'<',solve_channelSegments.get(k).getX2().add(b6.mul(model.M())));
                    model.post(solve_channelSegments.get(k).getX2(),'<',solve_channelSegments.get(k).getX1().add(b6.mul(model.M())));
                    model.post(solve_channelSegments.get(k).getX1().sub(this.margin).add(b6.mul(model.M())),'>',right);

                    model.post(solve_channelSegments.get(k).getX1(),'<',solve_channelSegments.get(k).getX2().add(b7.mul(model.M())));
                    model.post(solve_channelSegments.get(k).getX2(),'<',solve_channelSegments.get(k).getX1().add(b7.mul(model.M())));
                    model.post(solve_channelSegments.get(k).getX1().add(b7.mul(model.M())),'>',left);
                    model.post(solve_channelSegments.get(k).getX1().sub(b7.mul(model.M())),'<',right);
                    model.post(solve_channelSegments.get(k).getY1().sub(b7.mul(model.M())),'<',down);
                    model.post(solve_channelSegments.get(k).getY2().sub(b7.mul(model.M())),'<',down);

                    model.post(solve_channelSegments.get(k).getX1(),'<',solve_channelSegments.get(k).getX2().add(b8.mul(model.M())));
                    model.post(solve_channelSegments.get(k).getX2(),'<',solve_channelSegments.get(k).getX1().add(b8.mul(model.M())));
                    model.post(solve_channelSegments.get(k).getX1().add(b8.mul(model.M())),'>',left);
                    model.post(solve_channelSegments.get(k).getX1().sub(b8.mul(model.M())),'<',right);
                    model.post(solve_channelSegments.get(k).getY1().add(b8.mul(model.M())),'>',up);
                    model.post(solve_channelSegments.get(k).getY2().add(b8.mul(model.M())),'>',up);

                    for(int l = 0; l < solve_vias.size(); l++) {
                        BinaryVariable c1 = model.binVar("c1");
                        BinaryVariable c2 = model.binVar("c2");
                        BinaryVariable c3 = model.binVar("c3");
                        BinaryVariable c4 = model.binVar("c4");

                        model.post(c1.add(c2).add(c3).add(c4), '=', 3);

                        model.post(solve_vias.get(l).getX().sub(this.margin).add(c1.mul(model.M())), '>', solve_channelSegments.get(k).getX1());
                        model.post(solve_vias.get(l).getX().sub(this.margin).add(c1.mul(model.M())), '>', solve_channelSegments.get(k).getX2());

                        model.post(solve_vias.get(l).getX().add(this.margin).sub(c2.mul(model.M())), '<', solve_channelSegments.get(k).getX1());
                        model.post(solve_vias.get(l).getX().add(this.margin).sub(c2.mul(model.M())), '<', solve_channelSegments.get(k).getX2());

                        model.post(solve_vias.get(l).getY().sub(this.margin).add(c3.mul(model.M())), '>', solve_channelSegments.get(k).getY1());
                        model.post(solve_vias.get(l).getY().sub(this.margin).add(c3.mul(model.M())), '>', solve_channelSegments.get(k).getY2());

                        model.post(solve_vias.get(l).getY().add(this.margin).sub(c4.mul(model.M())), '<', solve_channelSegments.get(k).getY1());
                        model.post(solve_vias.get(l).getY().add(this.margin).sub(c4.mul(model.M())), '<', solve_channelSegments.get(k).getY2());
                    }
                }
            }

            for(int j = 0; j < solve_channelSegments.size(); j++) {
                for(int k = j + 1; k < solve_channelSegments.size(); k++) {
                    BinaryVariable b1 = model.binVar("b1");
                    BinaryVariable b2 = model.binVar("b2");
                    BinaryVariable b3 = model.binVar("b3");
                    BinaryVariable b4 = model.binVar("b4");
                    BinaryVariable b5 = model.binVar("b5");
                    BinaryVariable b6 = model.binVar("b6");
                    BinaryVariable b7 = model.binVar("b7");
                    BinaryVariable b8 = model.binVar("b8");
                    BinaryVariable b9 = model.binVar("b9");
                    BinaryVariable b10 = model.binVar("b10");

                    model.post(b1.add(b2).add(b3).add(b4).add(b5).add(b6).add(b7).add(b8).add(b9).add(b10), '=',9);

                    model.post(solve_channelSegments.get(j).getX1().add(b1.mul(model.M())),'>',solve_channelSegments.get(j).getX2());
                    model.post(solve_channelSegments.get(j).getX2().add(b1.mul(model.M())),'>',solve_channelSegments.get(j).getX1());
                    model.post(solve_channelSegments.get(k).getX1().add(b1.mul(model.M())),'>',solve_channelSegments.get(k).getX2());
                    model.post(solve_channelSegments.get(k).getX2().add(b1.mul(model.M())),'>',solve_channelSegments.get(k).getX1());
                    model.post(model.abs(solve_channelSegments.get(j).getX1().sub(solve_channelSegments.get(k).getX1())).add(b1.mul(model.M())), '>', this.margin);

                    model.post(solve_channelSegments.get(j).getY1().add(b2.mul(model.M())),'>',solve_channelSegments.get(j).getY2());
                    model.post(solve_channelSegments.get(j).getY2().add(b2.mul(model.M())),'>',solve_channelSegments.get(j).getY1());
                    model.post(solve_channelSegments.get(k).getY1().add(b2.mul(model.M())),'>',solve_channelSegments.get(k).getY2());
                    model.post(solve_channelSegments.get(k).getY2().add(b2.mul(model.M())),'>',solve_channelSegments.get(k).getY1());
                    model.post(model.abs(solve_channelSegments.get(j).getY1().sub(solve_channelSegments.get(k).getY1())).add(b2.mul(model.M())), '>', this.margin);

                    model.post(solve_channelSegments.get(j).getX1().add(b3.mul(model.M())),'>',solve_channelSegments.get(j).getX2());
                    model.post(solve_channelSegments.get(j).getX2().add(b3.mul(model.M())),'>',solve_channelSegments.get(j).getX1());
                    model.post(solve_channelSegments.get(k).getY1().add(b3.mul(model.M())),'>',solve_channelSegments.get(k).getY2());
                    model.post(solve_channelSegments.get(k).getY2().add(b3.mul(model.M())),'>',solve_channelSegments.get(k).getY1());
                    model.post(solve_channelSegments.get(j).getY1().add(this.margin), '<', solve_channelSegments.get(k).getY1().add(b3.mul(model.M())));
                    model.post(solve_channelSegments.get(j).getY2().add(this.margin), '<', solve_channelSegments.get(k).getY1().add(b3.mul(model.M())));

                    model.post(solve_channelSegments.get(j).getX1().add(b4.mul(model.M())),'>',solve_channelSegments.get(j).getX2());
                    model.post(solve_channelSegments.get(j).getX2().add(b4.mul(model.M())),'>',solve_channelSegments.get(j).getX1());
                    model.post(solve_channelSegments.get(k).getY1().add(b4.mul(model.M())),'>',solve_channelSegments.get(k).getY2());
                    model.post(solve_channelSegments.get(k).getY2().add(b4.mul(model.M())),'>',solve_channelSegments.get(k).getY1());
                    model.post(solve_channelSegments.get(j).getY1().sub(this.margin), '>', solve_channelSegments.get(k).getY1().sub(b4.mul(model.M())));
                    model.post(solve_channelSegments.get(j).getY2().sub(this.margin), '>', solve_channelSegments.get(k).getY1().sub(b4.mul(model.M())));

                    model.post(solve_channelSegments.get(j).getX1().add(b5.mul(model.M())),'>',solve_channelSegments.get(j).getX2());
                    model.post(solve_channelSegments.get(j).getX2().add(b5.mul(model.M())),'>',solve_channelSegments.get(j).getX1());
                    model.post(solve_channelSegments.get(k).getY1().add(b5.mul(model.M())),'>',solve_channelSegments.get(k).getY2());
                    model.post(solve_channelSegments.get(k).getY2().add(b5.mul(model.M())),'>',solve_channelSegments.get(k).getY1());
                    model.post(solve_channelSegments.get(k).getX1(), '<', solve_channelSegments.get(j).getX1().add(b5.mul(model.M())));
                    model.post(solve_channelSegments.get(k).getX2(), '<', solve_channelSegments.get(j).getX1().add(b5.mul(model.M())));

                    model.post(solve_channelSegments.get(j).getX1().add(b6.mul(model.M())),'>',solve_channelSegments.get(j).getX2());
                    model.post(solve_channelSegments.get(j).getX2().add(b6.mul(model.M())),'>',solve_channelSegments.get(j).getX1());
                    model.post(solve_channelSegments.get(k).getY1().add(b6.mul(model.M())),'>',solve_channelSegments.get(k).getY2());
                    model.post(solve_channelSegments.get(k).getY2().add(b6.mul(model.M())),'>',solve_channelSegments.get(k).getY1());
                    model.post(solve_channelSegments.get(k).getX1(), '>', solve_channelSegments.get(j).getX1().sub(b6.mul(model.M())));
                    model.post(solve_channelSegments.get(k).getX2(), '>', solve_channelSegments.get(j).getX1().sub(b6.mul(model.M())));

                    model.post(solve_channelSegments.get(k).getX1().add(b7.mul(model.M())),'>',solve_channelSegments.get(k).getX2());
                    model.post(solve_channelSegments.get(k).getX2().add(b7.mul(model.M())),'>',solve_channelSegments.get(k).getX1());
                    model.post(solve_channelSegments.get(j).getY1().add(b7.mul(model.M())),'>',solve_channelSegments.get(j).getY2());
                    model.post(solve_channelSegments.get(j).getY2().add(b7.mul(model.M())),'>',solve_channelSegments.get(j).getY1());
                    model.post(solve_channelSegments.get(k).getY1().add(this.margin), '<', solve_channelSegments.get(j).getY1().add(b7.mul(model.M())));
                    model.post(solve_channelSegments.get(k).getY2().add(this.margin), '<', solve_channelSegments.get(j).getY1().add(b7.mul(model.M())));

                    model.post(solve_channelSegments.get(k).getX1().add(b8.mul(model.M())),'>',solve_channelSegments.get(k).getX2());
                    model.post(solve_channelSegments.get(k).getX2().add(b8.mul(model.M())),'>',solve_channelSegments.get(k).getX1());
                    model.post(solve_channelSegments.get(j).getY1().add(b8.mul(model.M())),'>',solve_channelSegments.get(j).getY2());
                    model.post(solve_channelSegments.get(j).getY2().add(b8.mul(model.M())),'>',solve_channelSegments.get(j).getY1());
                    model.post(solve_channelSegments.get(k).getY1().sub(this.margin), '>', solve_channelSegments.get(j).getY1().sub(b8.mul(model.M())));
                    model.post(solve_channelSegments.get(k).getY2().sub(this.margin), '>', solve_channelSegments.get(j).getY1().sub(b8.mul(model.M())));

                    model.post(solve_channelSegments.get(k).getX1().add(b9.mul(model.M())),'>',solve_channelSegments.get(k).getX2());
                    model.post(solve_channelSegments.get(k).getX2().add(b9.mul(model.M())),'>',solve_channelSegments.get(k).getX1());
                    model.post(solve_channelSegments.get(j).getY1().add(b9.mul(model.M())),'>',solve_channelSegments.get(j).getY2());
                    model.post(solve_channelSegments.get(j).getY2().add(b9.mul(model.M())),'>',solve_channelSegments.get(j).getY1());
                    model.post(solve_channelSegments.get(j).getX1(), '<', solve_channelSegments.get(k).getX1().add(b9.mul(model.M())));
                    model.post(solve_channelSegments.get(j).getX2(), '<', solve_channelSegments.get(k).getX1().add(b9.mul(model.M())));

                    model.post(solve_channelSegments.get(k).getX1().add(b10.mul(model.M())),'>',solve_channelSegments.get(k).getX2());
                    model.post(solve_channelSegments.get(k).getX2().add(b10.mul(model.M())),'>',solve_channelSegments.get(k).getX1());
                    model.post(solve_channelSegments.get(j).getY1().add(b10.mul(model.M())),'>',solve_channelSegments.get(j).getY2());
                    model.post(solve_channelSegments.get(j).getY2().add(b10.mul(model.M())),'>',solve_channelSegments.get(j).getY1());
                    model.post(solve_channelSegments.get(j).getX1(), '>', solve_channelSegments.get(k).getX1().sub(b10.mul(model.M())));
                    model.post(solve_channelSegments.get(j).getX2(), '>', solve_channelSegments.get(k).getX1().sub(b10.mul(model.M())));
                }
            }
        }
    }
    public void solve() throws GRBException {
        model.solve();
    }

    public void output() throws GRBException {
        for(int i = 0; i < this.connection_wrapper.length; i++) {
            System.out.printf("第%d个connection\n", i+1);
            System.out.printf("via from layer%d to layer%d, x:%d y:%d\n", this.connection_wrapper[i].via.getFrom().getId(), this.connection_wrapper[i].via.getTo().getId(),
                    this.connection_wrapper[i].via.getX().getValue(), this.connection_wrapper[i].via.getY().getValue());
            for(int j = 0; j < NumOfSegments; j++) {
                System.out.printf("第%d段channel x1:%d y1:%d x2:%d y2:%d layer%d\n", j+1, this.connection_wrapper[i].channel_segment[j].getX1().getValue(),
                        this.connection_wrapper[i].channel_segment[j].getY1().getValue(), this.connection_wrapper[i].channel_segment[j].getX2().getValue(),
                        this.connection_wrapper[i].channel_segment[j].getY2().getValue(), this.connection_wrapper[i].channel_segment[j].getLayer().getId());
            }
        }
    }

    public Chip getChip() {return this.chip;}
    public ChipRequest getChip_request() {return this.chip_request;}
}
