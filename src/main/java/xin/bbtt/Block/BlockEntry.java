package xin.bbtt.Block;

import lombok.Data;
import java.util.List;

@Data
public class BlockEntry {
    private int id;
    private String name;
    private double hardness;
    private boolean diggable;
    private String material;
    private int minStateId;
    private int maxStateId;
    private int defaultState;
    private String boundingBox;
    private List<StateProperty> states;

    @Data
    public static class StateProperty {
        private String name;
        private String type;
        private int num_values;
        private List<String> values;

        public String getValueAt(int index) {
            if ("bool".equals(type)) {
                return Boolean.toString(index == 0);
            }
            if (values != null && index < values.size()) {
                return values.get(index);
            }
            return String.valueOf(index);
        }
    }
}
