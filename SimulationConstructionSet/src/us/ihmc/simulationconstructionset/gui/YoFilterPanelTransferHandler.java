package us.ihmc.simulationconstructionset.gui;

import javax.swing.*;
import java.awt.datatransfer.Transferable;

@SuppressWarnings("serial")
public class YoFilterPanelTransferHandler extends TransferHandler {
    public YoFilterPanelTransferHandler() {
        super();
    }

    @Override
    public boolean canImport(TransferSupport transferSupport) {
        return false;
    }

    @Override
    public boolean importData(TransferHandler.TransferSupport transferSupport) {
        return false;
    }

    @Override
    public int getSourceActions(JComponent c) {
        if (c instanceof AddYoFilterPanel) {
            return TransferHandler.COPY;
        }

        return TransferHandler.NONE;
    }

    @Override
    public Transferable createTransferable(JComponent c) {
        if (c instanceof AddYoFilterPanel) {
            Transferable tip = new YoFilterPanelTransferable();

            return tip;
        }

        return null;
    }

    @Override
    public void exportDone(JComponent c, Transferable t, int action) {
    }
}
