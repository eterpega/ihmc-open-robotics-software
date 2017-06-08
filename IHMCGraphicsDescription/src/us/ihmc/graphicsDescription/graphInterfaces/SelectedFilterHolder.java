package us.ihmc.graphicsDescription.graphInterfaces;

import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;
import java.util.ArrayList;

public class SelectedFilterHolder {
    private YoFilter filter;
    private final ArrayList<ChangeListener> listeners = new ArrayList<>();

    public SelectedFilterHolder() {
    }

    public void setSelectedFilter(YoFilter filter) {
        this.filter = filter;

        for (ChangeListener listener : listeners) {
            if (listener != null)
                listener.stateChanged(new ChangeEvent(this));
        }
    }

    public YoFilter getSelectedFilter() {
        return this.filter;
    }

    public void addChangeListener(ChangeListener listener) {
        listeners.add(listener);
    }
}
