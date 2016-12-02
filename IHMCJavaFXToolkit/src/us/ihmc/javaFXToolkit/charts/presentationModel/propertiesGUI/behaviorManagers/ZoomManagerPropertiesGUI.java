package us.ihmc.javaFXToolkit.charts.presentationModel.propertiesGUI.behaviorManagers;


import javafx.fxml.FXMLLoader;
import javafx.scene.control.TitledPane;
import us.ihmc.javaFXToolkit.charts.presentationModel.behaviorManagers.ZoomManager;
import us.ihmc.javaFXToolkit.charts.presentationModel.propertiesGUI.TitledPanePropertiesGUI;

import java.io.IOException;
import java.net.URL;

/**
 * Created by amoucheboeuf on 5/26/16.
 */
public class ZoomManagerPropertiesGUI implements TitledPanePropertiesGUI
{

   private ZoomManager chartZoomManager;
   private TitledPane titledPane;

   public ZoomManagerPropertiesGUI(ZoomManager chartZoomManager)
   {
      this.chartZoomManager = chartZoomManager;

      ZoomManagerPropertiesController controller = new ZoomManagerPropertiesController(this.chartZoomManager);

      try
      {
         final URL url = getClass().getResource("/xml/ZoomManagerProperties.fxml");
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