package us.ihmc.simulationconstructionset.gui;

import us.ihmc.graphicsDescription.graphInterfaces.SelectedFilterHolder;
import us.ihmc.graphicsDescription.graphInterfaces.YoFilter;

import javax.swing.*;
import java.awt.*;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;

public class AddYoFilterPanel extends JPanel implements MouseListener {
    SelectedFilterHolder selectedFilterHolder;
    YoFilter yoFilter;

    public AddYoFilterPanel(SelectedFilterHolder selectedFilterHolder, YoFilter yoFilter) {
        this.selectedFilterHolder = selectedFilterHolder;
        this.yoFilter = yoFilter;

        this.setTransferHandler(new YoFilterPanelTransferHandler());

        ImageIcon ii = new ImageIcon(yoFilter.getFilterIconPath());
        JLabel withIcon = new JLabel("", ii, JLabel.CENTER);
        withIcon.addMouseListener(this);
        this.add(withIcon);

        this.setPreferredSize(new Dimension(28, 28));

        this.addMouseListener(this);
    }

    @Override
    public void mouseClicked(MouseEvent mouseEvent) {
    }

    @Override
    public void mousePressed(MouseEvent event) {
        selectedFilterHolder.setSelectedFilter(yoFilter);
        this.getTransferHandler().exportAsDrag(this, event, TransferHandler.COPY);
        YoGraph.setSourceOfDrag(this);
    }

    @Override
    public void mouseReleased(MouseEvent mouseEvent) {

    }

    @Override
    public void mouseEntered(MouseEvent mouseEvent) {
    }

    @Override
    public void mouseExited(MouseEvent mouseEvent) {
    }
}
