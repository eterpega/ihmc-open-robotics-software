package us.ihmc.robotbuilder.gui.editors;

import javafx.beans.property.Property;
import javafx.scene.Node;
import javafx.scene.layout.GridPane;
import us.ihmc.robotbuilder.gui.Editor;

import javax.vecmath.Vector3d;
import java.util.Arrays;

import static us.ihmc.robotics.util.FunctionalObservableValue.functional;
import static us.ihmc.robotics.util.NoCycleProperty.noCycle;

/**
 *
 */
public class Vector3DEditor extends Editor<Vector3d>
{
   private final EditorComponent component;

   public Vector3DEditor(Property<Vector3d> valueProperty)
   {
      super(noCycle(valueProperty));
      component = new EditorComponent();
   }

   @Override public Node getEditor()
   {
      return component;
   }

   private class EditorComponent extends GridPane
   {
      @SuppressWarnings("unchecked")
      private final NumberField<Double>[] textFields = new NumberField[3];

      EditorComponent()
      {
         for (int i = 0; i < textFields.length; i++)
         {
            textFields[i] = new NumberField<>(Double.class);
            GridPane.setColumnIndex(textFields[i], i);
         }
         getChildren().addAll(textFields);

         functional(valueProperty()).consume(newValue ->
                                   {
                                      textFields[0].valueProperty().setValue(newValue.x);
                                      textFields[1].valueProperty().setValue(newValue.y);
                                      textFields[2].valueProperty().setValue(newValue.z);
                                   });

         Arrays.stream(textFields)
               .map(NumberField::valueProperty)
               .forEach(numberProperty -> numberProperty.addListener((observable, oldValue, newValue) -> valueProperty().setValue(getValue())));
      }

      void setValue(Vector3d value)
      {
         valueProperty().setValue(value);
      }

      Vector3d getValue()
      {
         Vector3d result = new Vector3d();
         result.x = textFields[0].getValue().orElse(0.0);
         result.y = textFields[1].getValue().orElse(0.0);
         result.z = textFields[2].getValue().orElse(0.0);
         return result;
      }
   }
}
