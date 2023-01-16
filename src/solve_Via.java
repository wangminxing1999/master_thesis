import chip.layer.Layer;
import dev.yushen.wrapperapi.gurobi.variable.IntegerVariable;

public class solve_Via {
    private Layer from;
    private Layer to;
    private IntegerVariable x;
    private IntegerVariable y;

    public solve_Via(Layer from, Layer to, IntegerVariable x, IntegerVariable y) {
        this.from = from;
        this.to = to;
        this.x = x;
        this.y = y;
    }

    public Layer getFrom() {
        return this.from;
    }

    public void setFrom(Layer from) {
        this.from = from;
    }

    public Layer getTo() {
        return this.to;
    }

    public void setTo(Layer to) {
        this.to = to;
    }

    public IntegerVariable getX() {
        return this.x;
    }

    public void setX(IntegerVariable x) {
        this.x = x;
    }

    public IntegerVariable getY() {
        return this.y;
    }

    public void setY(IntegerVariable y) {
        this.y = y;
    }
}
