import dev.yushen.wrapperapi.gurobi.solver.Model;
import dev.yushen.wrapperapi.gurobi.variable.BinaryVariable;
import dev.yushen.wrapperapi.gurobi.variable.IntegerVariable;
import gurobi.GRBException;

import java.util.Arrays;

public class Main {
    public static void main(String[] args) {
        int a[] = {1,2,3,4};
        for( int i = 0; i < a.length; i++) {
            a[i] ++;
        }
        System.out.println(a[0]);
    }
}