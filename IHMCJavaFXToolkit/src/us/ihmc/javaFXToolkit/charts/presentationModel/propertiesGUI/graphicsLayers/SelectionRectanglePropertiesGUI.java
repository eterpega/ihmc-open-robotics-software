package us.ihmc.javaFXToolkit.charts.presentationModel.propertiesGUI.graphicsLayers;

import javafx.fxml.FXMLLoader;
import javafx.scene.control.TitledPane;
import us.ihmc.javaFXToolkit.charts.presentationModel.graphicsLayers.SelectionRectangleGraphics;
import us.ihmc.javaFXToolkit.charts.presentationModel.propertiesGUI.TitledPanePropertiesGUI;

import java.io.IOException;
import java.net.URL;

/**
 * Created by amoucheboeuf on 5/31/16.
 */
public class SelectionRectanglePropertiesGUI implements TitledPanePropertiesGUI
{
   private TitledPane titledPane;
   private SelectionRectangleGraphics selectionRectangleGraphics;

   public SelectionRectanglePropertiesGUI(SelectionRectangleGraphics selectionRectangleGraphics)
   {
      this.selectionRectangleGraphics = selectionRectangleGraphics;
      SelectionRectanglePropertiesController controller = new SelectionRectanglePropertiesController(selectionRectangleGraphics);

      try
      {
         final URL url = getClass().getResource("/xml/SelectionRectangleProperties.fxml");
         final FXMLLoader fxmlLoader = new FXMLLoader(url);
         fxmlLoader.setController(controller);
         titledPane = (TitledPane) fxmlLoader.load();

      }
      catch (IOException ex)
      {
         System.err.println("Error loading: " + ex);
      }

   }


   @Override public TitledPane getTitledPane()
   {
      return titledPane;
   }
}
