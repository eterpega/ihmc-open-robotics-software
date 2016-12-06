package us.ihmc.robotbuilder.gui.editors;

import javafx.beans.property.Property;
import javafx.scene.Node;
import javafx.scene.control.ColorPicker;
import javafx.scene.layout.FlowPane;
import us.ihmc.robotbuilder.gui.Editor;

import java.awt.Color;

import static us.ihmc.robotics.util.FunctionalObservableValue.functional;
import static us.ihmc.robotics.util.NoCycleProperty.noCycle;

/**
 * An editor for {@link java.awt.Color} values.
 */
public class ColorEditor extends Editor<Color>
{
   private final FlowPane editorPane = new FlowPane();

   public ColorEditor(Property<Color> valueProperty)
   {
      super(noCycle(valueProperty));

      ColorPicker colorPicker = new ColorPicker();
      functional(valueProperty()).map(this::awtToJfx).consume(colorPicker::setValue);
      functional(colorPicker.valueProperty()).map(this::jfxToAwt).consume(valueProperty()::setValue);

      editorPane.getChildren().add(colorPicker);
   }

   private Color jfxToAwt(javafx.scene.paint.Color jfx)
   {
      return new Color((float)jfx.getRed(), (float)jfx.getGreen(), (float)jfx.getBlue());
   }

   private javafx.scene.paint.Color awtToJfx(Color awt)
   {
      return new javafx.scene.paint.Color(awt.getRed() / 255.0, awt.getGreen() / 255.0, awt.getBlue() / 255.0, 1);
   }

   @Override public Node getEditor()
   {
      return editorPane;
   }
}
