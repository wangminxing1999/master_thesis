//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

import chip.layer.Layer;
import java.util.ArrayList;
import java.util.List;

public class solve_Chip {
    private int height;
    private int width;
    private int zHeight;
    private List<solve_Layer> layers = new ArrayList();

    public solve_Chip() {
    }

    public int getHeight() {
        return this.height;
    }

    public void setHeight(int height) {
        this.height = height;
    }

    public int getWidth() {
        return this.width;
    }

    public void setWidth(int width) {
        this.width = width;
    }

    public int getzHeight() {
        return this.zHeight;
    }

    public void setzHeight(int zHeight) {
        this.zHeight = zHeight;
    }

    public List<solve_Layer> getLayers() {
        return this.layers;
    }

    public void setLayers(List<solve_Layer> layers) {
        this.layers = layers;
    }

    public void addLayer(solve_Layer layer) {
        this.layers.add(layer);
    }
}
