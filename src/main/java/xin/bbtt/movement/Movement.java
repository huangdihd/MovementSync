package xin.bbtt.movement;

import lombok.Getter;
import lombok.Setter;

@Setter
@Getter
public abstract class Movement {
    protected boolean finished = false;

    public abstract void init();
    public abstract void onTick();
    
    /**
     * @return the time limit in milliseconds. Return negative value to ignore time limit and rely on isFinished().
     */
    public abstract long getTime();
    
    public abstract void onStop();

    public void onPause() {
        // Default empty implementation, subclasses can override
    }

    public void onResume() {
        // Default empty implementation, subclasses can override
    }
}
