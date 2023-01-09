import chip.connection.ChannelSegment;
import chip.connection.Connection;
import chip.connection.Via;
import chip.module.Module;
import dev.yushen.wrapperapi.gurobi.solver.Model;
import dev.yushen.wrapperapi.gurobi.variable.BinaryVariable;
import dev.yushen.wrapperapi.gurobi.variable.IntegerVariable;
import gurobi.GRBException;
import chip.*;

import java.util.Arrays;
import java.util.List;

//未考虑当管道连续跨越多层时，中间层是否会有管道阻挡
//未考虑所有管道坐标都应该在芯片范围约束内
//未考虑速度约束，暂未引入管道半径
//未考虑各个元件之间应留有的距离余量

public class SolveConnection {
    private Chip chip;
    private ChipRequest chip_requirement;

    public SolveConnection() {}

    public void set_chip(Chip chip) {this.chip = chip;}

    public void set_chip_requirement(ChipRequest chip_requirement) {this.chip_requirement = chip_requirement;}

    public Chip get_chip() {return this.chip;}

    public ChipRequest get_chip_requirement() {return this.chip_requirement;}

    public ChannelSegment[] create_connection() {
        try {
            Model model = new Model("my Model");
            List<Connection> list = this.chip_requirement.getConnections();
            int total_num = list.size();
            //给每个connection分配10个ChannelSegment
            ChannelSegment[] connection_layout = new ChannelSegment[total_num * 10];
            IntegerVariable[] cs_x1 = model.intVarArray("cs_x1", total_num * 10, 0, chip.getWidth());
            IntegerVariable[] cs_x2 = model.intVarArray("cs_x2", total_num * 10, 0, chip.getWidth());
            IntegerVariable[] cs_y1 = model.intVarArray("cs_y1", total_num * 10, 0, chip.getHeight());
            IntegerVariable[] cs_y2 = model.intVarArray("cs_y2", total_num * 10, 0, chip.getHeight());
            int layer_id[] = new int [total_num * 10];
            int cycle = 0;
            for (Connection connection : list) {
                int layer_id_1 = connection.getPin1().getModule().getLayer().getId();
                int layer_id_2 = connection.getPin2().getModule().getLayer().getId();
                //首尾两个ChannelSegment的起点与终点应该与两个Pin的位置分别重合
                model.post(cs_x1[cycle * 10], '=', connection.getPin1().getX()) ;
                model.post(cs_y1[cycle * 10], '=', connection.getPin1().getY()) ;
                model.post(cs_x2[(cycle + 1) * 10 - 1], '=', connection.getPin2().getX()) ;
                model.post(cs_y2[(cycle + 1) * 10 - 1], '=', connection.getPin2().getY()) ;
                for (int i = cycle * 10; i < (cycle + 1) * 10; i++) {
                    connection_layout[i].setConnection(connection);
                    //线路只能是水平或竖直
                    BinaryVariable temp1 = model.binVar("b1");
                    BinaryVariable temp2 = model.binVar("b2");
                    model.post(cs_y1[i], '<', (cs_y2[i].add(temp1.mul(model.M()))));
                    model.post(cs_y2[i], '<', (cs_y1[i].add(temp1.mul(model.M()))));
                    model.post(cs_x1[i], '<', (cs_x2[i].add(temp2.mul(model.M()))));
                    model.post(cs_x2[i], '<', (cs_x1[i].add(temp2.mul(model.M()))));
                    model.post(temp1.add(temp2),'=', 1);
                    //相邻两个ChannelSegment的端点应该连接在一起（重合）
                    if(i != (cycle + 1) * 10 - 1) {
                        model.post(cs_x2[i], '=', cs_x1[i+1]);
                        model.post(cs_y2[i], '=', cs_y1[i+1]);
                    }
                    //每两个PIN中间用10个channelSegment连接，前五个在PIN1所在层，后五个在PIN2所在层
                    if(i < cycle * 10 + 5) {
                        layer_id[i] = layer_id_1;
                    } else {
                        layer_id[i] = layer_id_2;
                    }
                }
                cycle ++;
            }

            //ChannelSegment 与同层之间的module不能有重叠
            for(int i = 0; i < total_num * 10; i++) {
                List<Module> modules = this.chip.getLayers().get(layer_id[i]).getModules();
                for(Module module: modules) {
                    //根据Module四种不同角度设置上下左右界限
                    int left = 0, right = 0, up = 0, down = 0;
                    switch(module.getOrientation()) {
                        case D0 :
                            left = module.getX() - module.getWidth()/2;
                            right = module.getX() + module.getWidth()/2;
                            down = module.getY() - module.getHeight()/2;
                            up = module.getY() + module.getHeight()/2;
                        case D90 :
                            left = module.getX() - module.getHeight()/2;
                            right = module.getX() + module.getHeight()/2;
                            down = module.getY() - module.getWidth()/2;
                            up = module.getY() + module.getWidth()/2;
                        case D180 :
                            left = module.getX() - module.getWidth()/2;
                            right = module.getX() + module.getWidth()/2;
                            down = module.getY() - module.getHeight()/2;
                            up = module.getY() + module.getHeight()/2;
                        case D270 :
                            left = module.getX() - module.getHeight()/2;
                            right = module.getX() + module.getHeight()/2;
                            down = module.getY() - module.getWidth()/2;
                            up = module.getY() + module.getWidth()/2;
                    }
                    BinaryVariable r1 = model.binVar("r1");
                    BinaryVariable r2 = model.binVar("r2");
                    BinaryVariable r3 = model.binVar("r3");
                    BinaryVariable r4 = model.binVar("r4");
                    //四种不重叠情况上下左右
                    model.post(cs_x1[i].sub(r1.mul(model.M())), '<', left);
                    model.post(cs_x2[i].sub(r1.mul(model.M())), '<', left);
                    model.post(cs_x1[i].add(r2.mul(model.M())), '>', right);
                    model.post(cs_x2[i].add(r2.mul(model.M())), '>', right);
                    model.post(cs_y1[i].sub(r3.mul(model.M())), '<', down);
                    model.post(cs_y2[i].sub(r3.mul(model.M())), '<', down);
                    model.post(cs_y1[i].add(r4.mul(model.M())), '>', up);
                    model.post(cs_y2[i].add(r4.mul(model.M())), '>', up);
                }
                //ChannelSegment与同一平面内其他连接所包含的segment不能有交集
                int connection_index = i/10;
                for(int j = 0; j < total_num * 10; j++) {
                    if(j >= connection_index * 10 && j < (connection_index + 1) * 10)
                        continue;
                    if(layer_id[i] == layer_id[j]) {
                        BinaryVariable b1 = model.binVar("b1");
                        BinaryVariable b2 = model.binVar("b2");
                        BinaryVariable b3 = model.binVar("b3");
                        BinaryVariable b4 = model.binVar("b4");
                        model.post(b1.add(b2).add(b3).add(b4), '=',3);
                        //1
                        model.post(cs_x1[i],'<', cs_x1[j].add(b1.mul(model.M())));
                        model.post(cs_x1[i],'<', cs_x2[j].add(b1.mul(model.M())));
                        model.post(cs_x2[i],'<', cs_x1[j].add(b1.mul(model.M())));
                        model.post(cs_x2[i],'<', cs_x2[j].add(b1.mul(model.M())));
                        //2
                        model.post(cs_x1[i].add(b2.mul(model.M())), '>', cs_x1[j]);
                        model.post(cs_x1[i].add(b2.mul(model.M())), '>', cs_x2[j]);
                        model.post(cs_x2[i].add(b2.mul(model.M())), '>', cs_x1[j]);
                        model.post(cs_x2[i].add(b2.mul(model.M())), '>', cs_x2[j]);
                        //3
                        model.post(cs_y1[i],'<', cs_y1[j].add(b3.mul(model.M())));
                        model.post(cs_y1[i],'<', cs_y2[j].add(b3.mul(model.M())));
                        model.post(cs_y2[i],'<', cs_y1[j].add(b3.mul(model.M())));
                        model.post(cs_y2[i],'<', cs_y2[j].add(b3.mul(model.M())));
                        //4
                        model.post(cs_y1[i].add(b4.mul(model.M())), '>', cs_y1[j]);
                        model.post(cs_y1[i].add(b4.mul(model.M())), '>', cs_y2[j]);
                        model.post(cs_y2[i].add(b4.mul(model.M())), '>', cs_y1[j]);
                        model.post(cs_y2[i].add(b4.mul(model.M())), '>', cs_y2[j]);
                    }
                }
            }
            model.solve();
            for(int i = 0; i < total_num * 10; i++) {
                connection_layout[i].setX1(cs_x1[i].getValue());
                connection_layout[i].setY1(cs_y1[i].getValue());
                connection_layout[i].setX2(cs_x2[i].getValue());
                connection_layout[i].setY2(cs_y2[i].getValue());
                connection_layout[i].setLayer(this.chip.getLayers().get(layer_id[i]));
            }
            return connection_layout;
        } catch (GRBException e) {
            throw new RuntimeException(e);
        }
    }
}
