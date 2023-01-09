import chip.connection.ChannelSegment;
import chip.connection.Connection;
import chip.connection.Via;

public class ConnectionWrapper {
    private Connection connection;
    public ChannelSegment[] channel_segment;
    public Via via;
    public int NumberOfSegment;
    //Every connection consists of several channel segments and one(or zero) via
    public ConnectionWrapper(Connection connection, int NumberOfSegment) {
        this.connection = connection;
        this.NumberOfSegment = NumberOfSegment;
        this.channel_segment = new ChannelSegment[NumberOfSegment];
        this.via = new Via(null,null,0,0);
    }

    public Connection getConnection() {return this.connection;}
}
