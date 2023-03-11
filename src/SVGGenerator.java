import chip.Chip;
import chip.connection.ChannelSegment;
import chip.layer.Layer;
import chip.module.Module;
import chip.pin.Pin;

public class SVGGenerator {
    private Chip chip;

    private int width, height;

    public SVGGenerator(Chip chip) {
        this.chip = chip;
        this.width = chip.getWidth();
        this.height = chip.getHeight();
    }

    public String serializeLayer(int id){
        String svg_head = "<?xml version=\"1.0\"?>\n" +
                "<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.1//EN\" \n" +
                "    \"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd\">\n" +
                "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" \n" +
                "    width=\"" + width + "\" height=\"" + height +  "\" style=\"background-color:#868686\">\n";

        String svg_body = "";
        svg_body += "   <g class=\"COMPONENT\">\n";
        for(ChannelSegment cs : chip.getLayers().get(id).getChannelSegments()) {
            int x1 = cs.getX1(), y1 = cs.getY1(), x2 = cs.getX2(), y2 = cs.getY2();
            svg_body += "       <line x1=\"" + x1 + "\" y1=\"" + y1 + "\" x2=\"" + x2 + "\" y2=\"" + y2 + "\" style=\"stroke:rgb(255,0,0);stroke-width:1\" />\n";
        }
        for(Module module : chip.getLayers().get(id).getModules()){
            int x = module.getX(), y = module.getY();
            int w = module.getWidth(), h = module.getHeight();
            switch (module.getOrientation()){
                case D90, D270 -> {
                    w = module.getHeight();
                    h = module.getWidth();
                }
            }

            svg_body += "       <rect x=\"" + x + "\" y=\"" +y + "\" width=\"" +w + "\" height=\"" + h + "\" fill=\"#3E3E3E\"/>\n" +
                    "       <text x=\"" + (x + w / 2) + "\" y=\"" + (y + h / 2 + 2) + "\" font-family=\"Verdana\" font-size=\"1\" fill=\"#CECECE\" alignment-baseline=\"middle\" text-anchor=\"middle\">" +
                    module.getName() +
                    "</text>\n";

            for(Pin pin : module.getPins()){
                int pinX = module.getX() + pin.getX(), pinY = module.getY() + pin.getY();

                switch (module.getOrientation()){
                    case D90 -> {
                        pinX = module.getX() + pin.getY();
                        pinY = module.getY() + h - pin.getX();
                    }
                    case D180 -> {
                        pinX = module.getX() + w - pin.getX();
                        pinY = module.getY() + h - pin.getY();
                    }
                    case D270 -> {
                        pinX = module.getX() + w - pin.getY();
                        pinY = module.getY() + pin.getX();
                    }

                }
                svg_body += "       <circle cx=\"" + pinX  + "\" cy=\"" +pinY + "\" r=\"" + 1 + "\" fill=\"#FFF\"/>\n";
                svg_body += "       <text x=\"" + pinX + "\" y=\"" + pinY + "\" font-family=\"Verdana\" font-size=\"1\" fill=\"#4e6c9c\" alignment-baseline=\"middle\" text-anchor=\"middle\">"
                        + pin.getModule().getPins().indexOf(pin) +"</text>\n";
            }
        }
        svg_body += "   </g>";
        String svg_tail = "\n</svg>";

        return svg_head + svg_body + svg_tail;
    }
}
