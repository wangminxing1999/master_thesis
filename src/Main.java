import dev.yushen.wrapperapi.gurobi.solver.Model;
import dev.yushen.wrapperapi.gurobi.variable.BinaryVariable;
import gurobi.GRBException;

import java.util.Arrays;

public class Main {
    public static void main(String[] args) {
        try {
            Model s = new Model("s");
            BinaryVariable[] binaryVariables = s.binVarArray("ba", 5);

            s.post(s.sum( s.binVar("bax"), s.binVar("bax"), s.binVar("bax",1)),'=',1);

            s.solve();

            System.out.println(Arrays.asList(binaryVariables).toString());
        } catch (GRBException e) {
            throw new RuntimeException(e);
        }
    }
}