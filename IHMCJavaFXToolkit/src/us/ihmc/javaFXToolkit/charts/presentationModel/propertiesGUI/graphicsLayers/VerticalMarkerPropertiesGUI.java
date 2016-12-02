package us.ihmc.javaFXToolkit.charts.presentationModel.propertiesGUI.graphicsLayers;

import javafx.fxml.FXMLLoader;
import javafx.scene.control.TitledPane;
import us.ihmc.javaFXToolkit.charts.presentationModel.graphicsLayers.VerticalMarkerGraphics;
import us.ihmc.javaFXToolkit.charts.presentationModel.propertiesGUI.TitledPanePropertiesGUI;

import java.io.IOException;
import java.net.URL;

/**
 * Created by amoucheboeuf on 5/26/16.
 */
public class VerticalMarkerPropertiesGUI implements TitledPanePropertiesGUI
{
   private VerticalMarkerGraphics verticalMarkerGraphics;

   private TitledPane titledPane;

   public VerticalMarkerPropertiesGUI(VerticalMarkerGraphics verticalMarkerGraphics)
   {
      this.verticalMarkerGraphics = verticalMarkerGraphics;

      VerticalMarkerPropertiesController controller = new VerticalMarkerPropertiesController(this.verticalMarkerGraphics);

      try
      {
         final URL url = getClass().getResource("/xml/VerticalMarkerProperties.fxml");
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