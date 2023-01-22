//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

import chip.connection.ChannelSegment;
import chip.connection.Connection;
import chip.module.Module;
import java.util.ArrayList;
import java.util.List;

public class solve_Layer {
    private int id;
    private int level;
    private List<Module> modules;
    private List<Connection> connections;
    private List<solve_ChannelSegment> channelSegments;

    public solve_Layer(int id) {
        this.id = id;
        this.modules = new ArrayList();
        this.connections = new ArrayList();
        this.channelSegments = new ArrayList();
    }

    public int getLevel() {
        return this.level;
    }

    public void setLevel(int level) {
        this.level = level;
    }

    public int getId() {
        return this.id;
    }

    public void setId(int id) {
        this.id = id;
    }

    public void addModule(Module module) {
        this.modules.add(module);
    }

    public void addConnection(Connection connection) {
        this.connections.add(connection);
    }

    public void addChannel(solve_ChannelSegment channelSegment) {
        this.channelSegments.add(channelSegment);
    }

    public List<Module> getModules() {
        return this.modules;
    }

    public void setModules(List<Module> modules) {
        this.modules = modules;
    }

    public List<Connection> getConnections() {
        return this.connections;
    }

    public void setConnections(List<Connection> connections) {
        this.connections = connections;
    }

    public List<solve_ChannelSegment> getChannelSegments() {
        return this.channelSegments;
    }

    public void setChannelSegments(List<solve_ChannelSegment> channelSegments) {
        this.channelSegments = channelSegments;
    }
}
