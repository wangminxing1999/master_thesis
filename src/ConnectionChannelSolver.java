import chip.Chip;
import chip.connection.Connection;
import chip.connection.Via;
import chip.layer.Layer;
import chip.module.Module;
import chip.pin.Pin;
import dev.yushen.wrapperapi.gurobi.solver.Model;
import dev.yushen.wrapperapi.gurobi.variable.BinaryVariable;
import dev.yushen.wrapperapi.gurobi.variable.IntegerVariable;
import gurobi.GRBException;

import java.util.List;

import static java.lang.Math.max;
import static java.lang.Math.min;

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
        for(int i = 0; i < chip.getLayers().size(); i++) {
            layers_content_record[i] = new CSWandViaWPerLayer();
        }
        this.model = new Model("My model");
    }

    private void init_csw_per_connection(int begin, Connection conn, Layer layer, ChannelSegmentWrapper[] csw_group) throws GRBException {
        for(int k = begin; k < begin + NumOfSegments/2; k++) {
            IntegerVariable x1 = model.intVar("x1", 0, chip.getWidth());
            IntegerVariable y1 = model.intVar("y1", 0, chip.getHeight());
            IntegerVariable x2 = model.intVar("x2", 0, chip.getWidth());
            IntegerVariable y2 = model.intVar("y2", 0, chip.getHeight());
            csw_group[k] = new ChannelSegmentWrapper(x1,y1,x2,y2,conn,layer);
            layers_content_record[layer.getId()].add_csw(csw_group[k]);
            chip.getLayers().get(layer.getId()).addChannel(csw_group[k].getChannelSegment());
        }
    }

    private void init_vw_per_connection(Layer layer_pin1, Layer layer_pin2, ChannelSegmentWrapper[] csw_group) throws GRBException {
        IntegerVariable x = model.intVar("x",0, chip.getWidth());
        IntegerVariable y = model.intVar("y",0,chip.getHeight());
        ViaWrapper vw = new ViaWrapper(x,y,layer_pin1,layer_pin2);
        model.post(x,'=',csw_group[NumOfSegments/2].x1);
        model.post(y,'=',csw_group[NumOfSegments/2].y1);
        int m_min = min(layer_pin1.getId(),layer_pin2.getId());
        int m_max = max(layer_pin1.getId(),layer_pin2.getId());
        for(int i = m_min+1; i <= m_max-1; i++) {
            layers_content_record[i].add_viaw(vw);
        }
    }
    private void add_single_connection_channel_constraint(Pin p1, Pin p2, ChannelSegmentWrapper[] csw_group) throws GRBException {
        model.post(csw_group[0].x1,'=',p1.getX());
        model.post(csw_group[0].y1,'=',p1.getY());
        model.post(csw_group[NumOfSegments-1].x2,'=',p2.getX());
        model.post(csw_group[NumOfSegments-1].y2,'=',p2.getY());
        for(int i = 0; i < NumOfSegments; i++) {
            BinaryVariable b1 = model.binVar("b1");
            BinaryVariable b2 = model.binVar("b2");
            model.post(b1.add(b2),'=',1);
            model.post(csw_group[i].x1,'<',csw_group[i].x2.add(b1.mul(model.M())));
            model.post(csw_group[i].x2,'<',csw_group[i].x1.add(b1.mul(model.M())));
            model.post(csw_group[i].y1,'<',csw_group[i].y2.add(b2.mul(model.M())));
            model.post(csw_group[i].y2,'<',csw_group[i].y1.add(b2.mul(model.M())));
            if(i <= NumOfSegments - 2) {
                model.post(csw_group[i].x2,'=',csw_group[i+1].x1);
                model.post(csw_group[i].y2,'=',csw_group[i+1].y1);
            }
        }
    }
    public void init_single_connection_constraint() throws GRBException {
        for(int i = 0; i < chip.getLayers().size(); i++) {
            for(int j = 0; j < chip.getLayers().get(i).getConnections().size(); j++) {
                ChannelSegmentWrapper[] csw_group = new ChannelSegmentWrapper[NumOfSegments];
                Connection conn = chip.getLayers().get(i).getConnections().get(j);
                Pin pin1 = chip.getLayers().get(i).getConnections().get(j).getPin1();
                Pin pin2 = chip.getLayers().get(i).getConnections().get(j).getPin2();
                Layer layer_pin1 = pin1.getModule().getLayer();
                Layer layer_pin2 = pin2.getModule().getLayer();
                init_csw_per_connection(0,conn,layer_pin1,csw_group);
                init_csw_per_connection(NumOfSegments/2,conn,layer_pin2,csw_group);
                add_single_connection_channel_constraint(pin1, pin2, csw_group);
                if(layer_pin1 != layer_pin2) {
                    init_vw_per_connection(layer_pin1,layer_pin2,csw_group);
                }
            }
        }
    }

    public void avoid_collisions_at_same_layer() throws GRBException {
        for(int i = 0; i < chip.getLayers().size(); i++) {
            List<Module> modules = chip.getLayers().get(i).getModules();
            List<ChannelSegmentWrapper> csws = layers_content_record[i].get_csw();
            List<ViaWrapper> vws = layers_content_record[i].get_viaw();

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

                for(int k = 0; k < vws.size(); k++) {
                    BinaryVariable b1 = model.binVar("b1");
                    BinaryVariable b2 = model.binVar("b2");
                    BinaryVariable b3 = model.binVar("b3");
                    BinaryVariable b4 = model.binVar("b4");

                    model.post(b1.add(b2).add(b3).add(b4), '=', 3);

                    model.post(vws.get(k).x.add(this.margin).sub(b1.mul(model.M())), '<', left);
                    model.post(vws.get(k).x.sub(this.margin).add(b2.mul(model.M())), '>', right);
                    model.post(vws.get(k).y.add(this.margin).sub(b3.mul(model.M())), '<', down);
                    model.post(vws.get(k).y.sub(this.margin).add(b4.mul(model.M())), '>', up);
                }

                for(int k = 0; k < csws.size(); k++) {
                    BinaryVariable b1 = model.binVar("b1");
                    BinaryVariable b2 = model.binVar("b2");
                    BinaryVariable b3 = model.binVar("b3");
                    BinaryVariable b4 = model.binVar("b4");
                    BinaryVariable b5 = model.binVar("b5");
                    BinaryVariable b6 = model.binVar("b6");
                    BinaryVariable b7 = model.binVar("b7");
                    BinaryVariable b8 = model.binVar("b8");

                    model.post(b1.add(b2).add(b3).add(b4).add(b5).add(b6).add(b7).add(b8),'=',7);

                    model.post(csws.get(k).y1,'<',csws.get(k).y2.add(b1.mul(model.M())));
                    model.post(csws.get(k).y2,'<',csws.get(k).y1.add(b1.mul(model.M())));
                    model.post(csws.get(k).y1.add(this.margin).sub(b1.mul(model.M())),'<',down);

                    model.post(csws.get(k).y1,'<',csws.get(k).y2.add(b2.mul(model.M())));
                    model.post(csws.get(k).y2,'<',csws.get(k).y1.add(b2.mul(model.M())));
                    model.post(csws.get(k).y1.sub(this.margin).add(b2.mul(model.M())),'>',up);

                    model.post(csws.get(k).y1,'<',csws.get(k).y2.add(b3.mul(model.M())));
                    model.post(csws.get(k).y2,'<',csws.get(k).y1.add(b3.mul(model.M())));
                    model.post(csws.get(k).y1.add(b3.mul(model.M())),'>',down);
                    model.post(csws.get(k).y1.sub(b3.mul(model.M())),'<',up);
                    model.post(csws.get(k).x1.sub(b3.mul(model.M())),'<',left);
                    model.post(csws.get(k).x2.sub(b3.mul(model.M())),'<',left);

                    model.post(csws.get(k).y1,'<',csws.get(k).y2.add(b4.mul(model.M())));
                    model.post(csws.get(k).y2,'<',csws.get(k).y1.add(b4.mul(model.M())));
                    model.post(csws.get(k).y1.add(b4.mul(model.M())),'>',down);
                    model.post(csws.get(k).y1.sub(b4.mul(model.M())),'<',up);
                    model.post(csws.get(k).x1.add(b4.mul(model.M())),'>',right);
                    model.post(csws.get(k).x2.add(b4.mul(model.M())),'>',right);

                    model.post(csws.get(k).x1,'<',csws.get(k).x2.add(b5.mul(model.M())));
                    model.post(csws.get(k).x2,'<',csws.get(k).x1.add(b5.mul(model.M())));
                    model.post(csws.get(k).x1.add(this.margin).sub(b5.mul(model.M())),'<',left);

                    model.post(csws.get(k).x1,'<',csws.get(k).x2.add(b6.mul(model.M())));
                    model.post(csws.get(k).x2,'<',csws.get(k).x1.add(b6.mul(model.M())));
                    model.post(csws.get(k).x1.sub(this.margin).add(b6.mul(model.M())),'>',right);

                    model.post(csws.get(k).x1,'<',csws.get(k).x2.add(b7.mul(model.M())));
                    model.post(csws.get(k).x2,'<',csws.get(k).x1.add(b7.mul(model.M())));
                    model.post(csws.get(k).x1.add(b7.mul(model.M())),'>',left);
                    model.post(csws.get(k).x1.sub(b7.mul(model.M())),'<',right);
                    model.post(csws.get(k).y1.sub(b7.mul(model.M())),'<',down);
                    model.post(csws.get(k).y2.sub(b7.mul(model.M())),'<',down);

                    model.post(csws.get(k).x1,'<',csws.get(k).x2.add(b8.mul(model.M())));
                    model.post(csws.get(k).x2,'<',csws.get(k).x1.add(b8.mul(model.M())));
                    model.post(csws.get(k).x1.add(b8.mul(model.M())),'>',left);
                    model.post(csws.get(k).x1.sub(b8.mul(model.M())),'<',right);
                    model.post(csws.get(k).y1.add(b8.mul(model.M())),'>',up);
                    model.post(csws.get(k).y2.add(b8.mul(model.M())),'>',up);
                }
            }

            for(int j = 0; j < csws.size(); j++) {
                for(int k = j + 1; k < csws.size(); k++) {
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

                    model.post(csws.get(j).x1.add(b1.mul(model.M())),'>',csws.get(j).x2);
                    model.post(csws.get(j).x2.add(b1.mul(model.M())),'>',csws.get(j).x1);
                    model.post(csws.get(k).x1.add(b1.mul(model.M())),'>',csws.get(k).x2);
                    model.post(csws.get(k).x2.add(b1.mul(model.M())),'>',csws.get(k).x1);
                    model.post(model.abs(csws.get(j).x1.sub(csws.get(k).x1)).add(b1.mul(model.M())), '>', this.margin);

                    model.post(csws.get(j).y1.add(b2.mul(model.M())),'>',csws.get(j).y2);
                    model.post(csws.get(j).y2.add(b2.mul(model.M())),'>',csws.get(j).y1);
                    model.post(csws.get(k).y1.add(b2.mul(model.M())),'>',csws.get(k).y2);
                    model.post(csws.get(k).y2.add(b2.mul(model.M())),'>',csws.get(k).y1);
                    model.post(model.abs(csws.get(j).y1.sub(csws.get(k).y1)).add(b2.mul(model.M())), '>', this.margin);

                    model.post(csws.get(j).x1.add(b3.mul(model.M())),'>',csws.get(j).x2);
                    model.post(csws.get(j).x2.add(b3.mul(model.M())),'>',csws.get(j).x1);
                    model.post(csws.get(k).y1.add(b3.mul(model.M())),'>',csws.get(k).y2);
                    model.post(csws.get(k).y2.add(b3.mul(model.M())),'>',csws.get(k).y1);
                    model.post(csws.get(j).y1.add(this.margin), '<', csws.get(k).y1.add(b3.mul(model.M())));
                    model.post(csws.get(j).y2.add(this.margin), '<', csws.get(k).y1.add(b3.mul(model.M())));

                    model.post(csws.get(j).x1.add(b4.mul(model.M())),'>',csws.get(j).x2);
                    model.post(csws.get(j).x2.add(b4.mul(model.M())),'>',csws.get(j).x1);
                    model.post(csws.get(k).y1.add(b4.mul(model.M())),'>',csws.get(k).y2);
                    model.post(csws.get(k).y2.add(b4.mul(model.M())),'>',csws.get(k).y1);
                    model.post(csws.get(j).y1.sub(this.margin), '>', csws.get(k).y1.sub(b4.mul(model.M())));
                    model.post(csws.get(j).y2.sub(this.margin), '>', csws.get(k).y1.sub(b4.mul(model.M())));

                    model.post(csws.get(j).x1.add(b5.mul(model.M())),'>',csws.get(j).x2);
                    model.post(csws.get(j).x2.add(b5.mul(model.M())),'>',csws.get(j).x1);
                    model.post(csws.get(k).y1.add(b5.mul(model.M())),'>',csws.get(k).y2);
                    model.post(csws.get(k).y2.add(b5.mul(model.M())),'>',csws.get(k).y1);
                    model.post(csws.get(k).x1, '<', csws.get(j).x1.add(b5.mul(model.M())));
                    model.post(csws.get(k).x2, '<', csws.get(j).x1.add(b5.mul(model.M())));

                    model.post(csws.get(j).x1.add(b6.mul(model.M())),'>',csws.get(j).x2);
                    model.post(csws.get(j).x2.add(b6.mul(model.M())),'>',csws.get(j).x1);
                    model.post(csws.get(k).y1.add(b6.mul(model.M())),'>',csws.get(k).y2);
                    model.post(csws.get(k).y2.add(b6.mul(model.M())),'>',csws.get(k).y1);
                    model.post(csws.get(k).x1, '>', csws.get(j).x1.sub(b6.mul(model.M())));
                    model.post(csws.get(k).x2, '>', csws.get(j).x1.sub(b6.mul(model.M())));

                    model.post(csws.get(k).x1.add(b7.mul(model.M())),'>',csws.get(k).x2);
                    model.post(csws.get(k).x2.add(b7.mul(model.M())),'>',csws.get(k).x1);
                    model.post(csws.get(j).y1.add(b7.mul(model.M())),'>',csws.get(j).y2);
                    model.post(csws.get(j).y2.add(b7.mul(model.M())),'>',csws.get(j).y1);
                    model.post(csws.get(k).y1.add(this.margin), '<', csws.get(j).y1.add(b7.mul(model.M())));
                    model.post(csws.get(k).y2.add(this.margin), '<', csws.get(j).y1.add(b7.mul(model.M())));

                    model.post(csws.get(k).x1.add(b8.mul(model.M())),'>',csws.get(k).x2);
                    model.post(csws.get(k).x2.add(b8.mul(model.M())),'>',csws.get(k).x1);
                    model.post(csws.get(j).y1.add(b8.mul(model.M())),'>',csws.get(j).y2);
                    model.post(csws.get(j).y2.add(b8.mul(model.M())),'>',csws.get(j).y1);
                    model.post(csws.get(k).y1.sub(this.margin), '>', csws.get(j).y1.sub(b8.mul(model.M())));
                    model.post(csws.get(k).y2.sub(this.margin), '>', csws.get(j).y1.sub(b8.mul(model.M())));

                    model.post(csws.get(k).x1.add(b9.mul(model.M())),'>',csws.get(k).x2);
                    model.post(csws.get(k).x2.add(b9.mul(model.M())),'>',csws.get(k).x1);
                    model.post(csws.get(j).y1.add(b9.mul(model.M())),'>',csws.get(j).y2);
                    model.post(csws.get(j).y2.add(b9.mul(model.M())),'>',csws.get(j).y1);
                    model.post(csws.get(j).x1, '<', csws.get(k).x1.add(b9.mul(model.M())));
                    model.post(csws.get(j).x2, '<', csws.get(k).x1.add(b9.mul(model.M())));

                    model.post(csws.get(k).x1.add(b10.mul(model.M())),'>',csws.get(k).x2);
                    model.post(csws.get(k).x2.add(b10.mul(model.M())),'>',csws.get(k).x1);
                    model.post(csws.get(j).y1.add(b10.mul(model.M())),'>',csws.get(j).y2);
                    model.post(csws.get(j).y2.add(b10.mul(model.M())),'>',csws.get(j).y1);
                    model.post(csws.get(j).x1, '>', csws.get(k).x1.sub(b10.mul(model.M())));
                    model.post(csws.get(j).x2, '>', csws.get(k).x1.sub(b10.mul(model.M())));
                }
            }
        }
    }

    public void solve() throws GRBException {
        init_single_connection_constraint();
        avoid_collisions_at_same_layer();
        model.solve();
        for(int i = 0; i < chip.getLayers().size(); i++) {
            for(int j = 0; j < layers_content_record[i].get_csw_size(); j++) {
                layers_content_record[i].get_csw().get(j).fill_parameter();
            }
            for(int j = 0; j < layers_content_record[i].get_viaw_size(); j++) {
                layers_content_record[i].get_viaw().get(j).fill_parameter();
            }
        }
    }
}
