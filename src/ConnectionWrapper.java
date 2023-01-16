import chip.connection.ChannelSegment;
import chip.connection.Connection;
import chip.connection.Via;
import dev.yushen.wrapperapi.gurobi.variable.IntegerVariable;

public class ConnectionWrapper {
    private Connection connection;
    public solve_ChannelSegment[] channel_segment;
    public solve_Via via;
    public int NumberOfSegment;
    //Every connection consists of several channel segments and one(or zero) via
    public ConnectionWrapper(Connection connection, int NumberOfSegment) {
        this.connection = connection;
        this.NumberOfSegment = NumberOfSegment;
        this.channel_segment = new solve_ChannelSegment[NumberOfSegment];
    }

    public Connection getConnection() {return this.connection;}
}
