package xin.bbtt.pathfinding;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockStateParser;
import xin.bbtt.Block.UnknownBlockStateException;

class BlockStateLookupTest {
    @Test
    void indexedLookupMatchesRangeLookupForEveryRegisteredState() {
        BlockStateParser parser = BlockStateParser.Instance;
        for (int stateId = 0; stateId < BlockStateParser.getBlockStateRegistrySize(); stateId++) {
            var entry = parser.getBlockByStateId(stateId);
            var state = parser.parseStateId(stateId);
            assertEquals(entry.getName(), state.blockName());
            assertEquals(stateId, state.stateId());
            assertEquals(entry.getBoundingBox(), state.boundingBox());
            assertEquals(entry.getHardness(), state.hardness());
            assertEquals(entry.isDiggable(), state.diggable());
            assertEquals(entry.getMaterial(), state.material());
            int offset = stateId - entry.getMinStateId();
            var properties = entry.getStates();
            if (properties == null) {
                assertTrue(state.properties().isEmpty());
                continue;
            }
            assertEquals(properties.size(), state.properties().size());
            for (int index = properties.size() - 1; index >= 0; index--) {
                var property = properties.get(index);
                assertEquals(property.getValueAt(offset % property.getNum_values()), state.getProperty(property.getName()));
                offset /= property.getNum_values();
            }
        }
    }

    @Test
    void invalidIdsStillFailAndReturnedPropertiesRemainIndependent() {
        BlockStateParser parser = BlockStateParser.Instance;
        assertThrows(UnknownBlockStateException.class, () -> parser.parseStateId(-1));
        assertThrows(UnknownBlockStateException.class, () -> parser.parseStateId(Integer.MAX_VALUE));
        var first = parser.parseStateId(1);
        first.properties().put("test", "changed");
        assertNull(parser.parseStateId(1).getProperty("test"));
    }
}
