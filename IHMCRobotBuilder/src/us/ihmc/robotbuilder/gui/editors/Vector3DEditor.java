package us.ihmc.robotbuilder.gui.editors;

import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ObservableValue;
import javafx.scene.layout.GridPane;
import org.controlsfx.control.PropertySheet.Item;
import org.controlsfx.property.editor.AbstractPropertyEditor;
import us.ihmc.robotbuilder.gui.editors.Vector3DEditor.EditorComponent;

import javax.vecmath.Vector3d;
import java.util.Arrays;

/**
 *
 */
public class Vector3DEditor extends AbstractPropertyEditor<Vector3d, EditorComponent>
{
   public Vector3DEditor(Item property)
   {
      super(property, new EditorComponent());
   }

   @Override protected ObservableValue<Vector3d> getObservableValue()
   {
      return getEditor().valueProperty();
   }

   @Override public void setValue(Vector3d value)
   {
      getEditor().setValue(value);
   }

   static class EditorComponent extends GridPane
   {
      @SuppressWarnings("unchecked")
      private final NumberField<Double>[] textFields = new NumberField[3];
      private final Property<Vector3d> valueProperty = new SimpleObjectProperty<>();
      private boolean ignoreEdit = false;

      EditorComponent()
      {
         for (int i = 0; i < textFields.length; i++)
         {
            textFields[i] = new NumberField<>(Double.class);
            GridPane.setColumnIndex(textFields[i], i);
         }
         getChildren().addAll(textFields);

         valueProperty.addListener((observable, oldValue, newValue) ->
                                   {
                                      if (!ignoreEdit)
                                      {
                                         ignoreEdit = true;
                                         textFields[0].setText(Double.toString(newValue.x));
                                         textFields[1].setText(Double.toString(newValue.y));
                                         textFields[2].setText(Double.toString(newValue.z));
                                         ignoreEdit = false;
                                      }
                                   });

         Arrays.stream(textFields)
               .map(NumberField::valueProperty)
               .forEach(textProperty -> textProperty.addListener((observable, oldValue, newValue) ->
                                        {
                                           if (ignoreEdit)
                                              return;
                                           ignoreEdit = true;
                                           valueProperty.setValue(getValue());
                                           ignoreEdit = false;
                                        }));
      }

      Property<Vector3d> valueProperty()
      {
         return valueProperty;
      }

      void setValue(Vector3d value)
      {
         valueProperty.setValue(value);
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
