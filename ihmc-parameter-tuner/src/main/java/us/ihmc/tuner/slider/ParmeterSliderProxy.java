package us.ihmc.tuner.slider;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicReference;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.input.Dragboard;
import javafx.scene.input.TransferMode;
import javafx.scene.layout.Pane;
import us.ihmc.robotics.dataStructures.parameter.*;
import us.ihmc.tuner.slider.config.*;
import us.ihmc.tuner.util.ParameterNameUtil;

/**
 * Allows a "slider box" in the pane to display different types of sliders. For example, an empty slider can be displayed when no parameter is available, and a
 * numerical slider can be displayed once a parameter is dropped.
 */
public class ParmeterSliderProxy extends Pane
{
   private static final String FXML_PATH = "param_slider_proxy.fxml";

   @FXML private Pane root;

   public ParmeterSliderProxy() throws IOException
   {
      FXMLLoader loader = new FXMLLoader(getClass().getResource(FXML_PATH));
      loader.setRoot(this);
      loader.setController(this);
      loader.load();
   }

   public void initialize() throws IOException
   {
      // Bind drag/drop on root pane
      root.setOnDragOver(event ->
      {
         if (!event.getGestureSource().equals(root) && event.getDragboard().hasString())
         {
            event.acceptTransferModes(TransferMode.LINK);
         }

         event.consume();
      });
      root.setOnDragDropped(event ->
      {
         Dragboard dragboard = event.getDragboard();

         String draggablePath = dragboard.getString();
         String parameterName = ParameterNameUtil.getParameterNameFromDraggablePath(draggablePath);
         int parameterIdx = ParameterNameUtil.getParameterIndexFromDraggablePath(draggablePath);

         Parameter parameter = ParameterRegistry.getInstance().getParameter(parameterName);
         if (parameter != null)
         {
            try
            {
               SliderConfiguration config = configure(parameter, parameterIdx);
               if (config != null)
               {
                  swapChild(new NumericParameterSlider(parameter, config));
               }
            }
            catch (IOException e)
            {
               throw new RuntimeException(e);
            }
         }
      });

      swapChild(new EmptyParameterSlider());
   }

   private void swapChild(Pane newChild)
   {
      root.getChildren().setAll(newChild);
   }

   private SliderConfiguration configure(Parameter parameter, final int parameterIdx)
   {
      // atomic for mutability in inner-class
      final AtomicReference<SliderConfiguration> sliderConfiguration = new AtomicReference<>();
      parameter.accept(new ParameterVisitor()
      {
         @Override
         public void visitBoolean(BooleanParameter parameter)
         {
            sliderConfiguration.set(new BooleanSliderConfiguration());
         }

         @Override
         public void visitDoubleArray(DoubleArrayParameter parameter)
         {
            sliderConfiguration.set(new DoubleArraySliderConfiguration(parameterIdx, parameter.get(parameterIdx)));
         }

         @Override
         public void visitDouble(DoubleParameter parameter)
         {
            sliderConfiguration.set(new DoubleSliderConfiguration(parameter.get()));
         }

         @Override
         public void visitIntegerArray(IntegerArrayParameter parameter)
         {
            sliderConfiguration.set(new IntegerArraySliderConfiguration(parameterIdx, parameter.get(parameterIdx)));
         }

         @Override
         public void visitInteger(IntegerParameter parameter)
         {
            sliderConfiguration.set(new IntegerSliderConfiguration(parameter.get()));
         }

         @Override
         public void visitString(StringParameter parameter)
         {

         }
      });

      return sliderConfiguration.get();
   }
}
