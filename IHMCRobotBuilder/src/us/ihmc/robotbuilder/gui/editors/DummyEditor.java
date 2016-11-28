package us.ihmc.robotbuilder.gui.editors;

import javafx.beans.property.Property;
import javafx.scene.Node;
import javafx.scene.control.Label;
import us.ihmc.robotbuilder.gui.Editor;

import static us.ihmc.robotics.util.FunctionalObservableValue.functional;

/**
 * A dummy implementation of an editor that does not allow any editing and only
 * displays the current value.
 */
public class DummyEditor<T> extends Editor<T>
{
   private final Label label = new Label();

   public DummyEditor(Property<T> valueProperty)
   {
      super(valueProperty);
      label.textProperty().bind(functional(valueProperty).map(Object::toString));
   }

   @Override public Node getEditor()
   {
      return label;
   }
}
