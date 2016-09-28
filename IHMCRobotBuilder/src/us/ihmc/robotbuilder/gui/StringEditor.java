package us.ihmc.robotbuilder.gui;

import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;
import javafx.scene.control.Tooltip;
import javafx.scene.layout.Pane;

/**
 *
 */
public class StringEditor extends Editor<String>
{
   private final TextField textField = new TextField();

   public StringEditor(String label, String help)
   {
      super(label, help);

      getChildren().add(new Label(label));
      getChildren().add(textField);
      if (!help.isEmpty())
         textField.setTooltip(new Tooltip(help));
   }

   @Override public Class<String> getValueType()
   {
      return String.class;
   }

   @Override public Property<String> valueProperty()
   {
      return textField.textProperty();
   }
}
