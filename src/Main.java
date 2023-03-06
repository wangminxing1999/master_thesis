import chip.Chip;
import chip.ChipRequest;
import chip.connection.Connection;
import chip.layer.Layer;
import chip.module.Filter;
import chip.module.Module;
import chip.module.Orientation;
import chip.pin.Pin;
import gurobi.GRBException;

import java.util.ArrayList;
import java.util.List;

public class Main {
    public static void main(String[] args) throws GRBException {
        Chip chip = new Chip();
        chip.setHeight(100);
        chip.setWidth(100);
        chip.setzHeight(100);
        List<Layer> layers = new ArrayList<Layer>();
        Layer l0 = new Layer(0);
        Layer l1 = new Layer(1);
        Layer l2 = new Layer(2);
        layers.add(l0);
        layers.add(l1);
        layers.add(l2);
        chip.setLayers(layers);
        Module m1 = new Filter("m1", 10 , 20);
        Module m2 = new Filter("m2", 20, 16);
        Module m3 = new Filter("m3", 30, 20);
        Pin p1 = new Pin(15,25);
        Pin p2 = new Pin(60,50);
        Pin p3 = new Pin(20,25);
        Pin p4 = new Pin(60,56);
        Pin p5 = new Pin(15,50);
        Pin p6 = new Pin(60,42);
        m1.setLayer(l0);
        m1.setX(15);
        m1.setY(15);
        m1.setOrientation(Orientation.D0);
        m1.addPin(p1);
        m1.addPin(p3);
        m2.setLayer(l0);
        m2.setX(50);
        m2.setY(50);
        m2.setOrientation(Orientation.D0);
        m2.addPin(p2);
        m2.addPin(p4);
        m2.addPin(p6);
        m3.setLayer(l1);
        m3.setX(30);
        m3.setY(50);
        m3.setOrientation(Orientation.D0);
        m3.addPin(p5);
        List<Module> modules1 = new ArrayList<Module>();
        List<Module> modules2 = new ArrayList<Module>();
        modules1.add(m1);
        modules1.add(m2);
        modules2.add(m3);
        l0.setModules(modules1);
        l1.setModules(modules2);
        Connection connection1 = new Connection(p1,p2);
        Connection connection2 = new Connection(p3,p4);
        Connection connection3 = new Connection(p5,p6);
        ChipRequest chiprequest = new ChipRequest();
        chiprequest.addConnection(connection1);
        chiprequest.addConnection(connection2);
        chiprequest.addConnection(connection3);
        CS cs = new CS(chip,chiprequest,4,1);
        cs.add_channelsegment_self_constraint();
        cs.add_avoid_collision_against_modules_or_each_other();
        cs.solve();
        cs.output();

    }
}