package us.ihmc.robotbuilder.gui.editors;

import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ObservableValue;
import javafx.scene.Node;
import javafx.scene.control.Button;
import javafx.scene.control.TextField;
import javafx.scene.control.TitledPane;
import javafx.scene.layout.GridPane;
import org.controlsfx.control.PropertySheet.Item;
import org.controlsfx.property.editor.AbstractPropertyEditor;
import org.controlsfx.property.editor.PropertyEditor;

import java.util.*;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import java.util.stream.Stream;

/**
 *
 */
public class IterableEditor<T> extends AbstractPropertyEditor<Iterable<T>, TitledPane>
{
   private final GridPane editorGrid = new GridPane();
   private final EditorFactory editorFactory;
   private List<T> value = new ArrayList<>();
   private Property<Iterable<T>> valueProperty;

   public IterableEditor(Item property, EditorFactory editorFactory)
   {
      super(property, new TitledPane());
      this.editorFactory = editorFactory;

      TitledPane editorContainer = getEditor();
      editorContainer.setExpanded(false);
      editorContainer.setText("Click to expand");
      editorContainer.setContent(editorGrid);
      editorContainer.setAnimated(false);
   }

   @Override protected ObservableValue<Iterable<T>> getObservableValue()
   {
      if (valueProperty == null)
         valueProperty = new SimpleObjectProperty<>(Collections.unmodifiableCollection(value == null ? new ArrayList<>() : value));
      return valueProperty;
   }

   @Override public Iterable<T> getValue()
   {
      return value;
   }

   @Override public void setValue(Iterable<T> value)
   {
      editorGrid.getChildren().clear();
      this.value = new ArrayList<>();
      for (T t : value)
      {
         this.value.add(t);
      }
      valueProperty.setValue(Collections.unmodifiableCollection(this.value));

      List<ListItemEditor> editors = this.value.stream()
                                               .map(ListItemEditor::new)
                                               .collect(Collectors.toList());

      int index = 0;
      for (ListItemEditor editor : editors)
      {
         Node editorNode = editor.getEditor();
         GridPane.setRowIndex(editorNode, index++);
         editorGrid.getChildren().add(editorNode);
      }

      Button addButton = new Button("Add");
      GridPane.setRowIndex(addButton, index);
      editorGrid.getChildren().add(addButton);

   }

   private class ListItemEditor implements PropertyEditor<T>
   {
      private final Button remove = new Button("\uf00d");
      private final Button moveUp = new Button("\uf077");
      private final Button moveDown = new Button("\uf078");
      private T originalValue;
      private PropertyEditor<T> innerEditor;
      private final GridPane container = new GridPane();

      ListItemEditor(T originalValue)
      {
         this.originalValue = originalValue;
         ItemProperty property = new ItemProperty(this::getValue, this::setValue);
         //noinspection unchecked
         innerEditor = (PropertyEditor<T>)editorFactory.call(property);
         if (innerEditor == null)
            innerEditor = new DefaultEditor<>(property);

         Node[] nodes = {remove, moveUp, moveDown, innerEditor.getEditor()};
         for (int i = 0; i < nodes.length; i++)
         {
            GridPane.setColumnIndex(nodes[i], i);
         }
         container.getChildren().addAll(nodes);

         remove.setOnAction(event ->
                            {
                               editorGrid.getChildren().remove(container);
                               value.remove(originalValue);
                               updateValue();
                            });

         moveUp.setOnAction(event -> {
            int index = value.indexOf(originalValue);
            if (index > 0)
            {
               value.set(index, value.get(index - 1));
               value.set(index - 1, originalValue);
               swapEditors(index - 1, index);
               updateValue();
            }
         });

         moveDown.setOnAction(event -> {
            int index = value.indexOf(originalValue);
            if (index >= 0 && index < value.size() - 1)
            {
               value.set(index, value.get(index + 1));
               value.set(index + 1, originalValue);
               swapEditors(index, index + 1);
               updateValue();
            }
         });

         Stream.of(moveUp, moveDown, remove).forEach(button -> button.setStyle("-fx-font-family: \"FontAwesome\";"));
      }

      private void updateValue()
      {
         valueProperty.setValue(Collections.unmodifiableCollection(IterableEditor.this.value));
      }

      private void swapEditors(int index1, int index2)
      {
         GridPane.setRowIndex(editorGrid.getChildren().get(index1), index2);
         GridPane.setRowIndex(editorGrid.getChildren().get(index2), index1);
         Node tmp1 = editorGrid.getChildren().get(index1);
         Node tmp2 = editorGrid.getChildren().get(index2);
         editorGrid.getChildren().set(index2, new GridPane());
         editorGrid.getChildren().set(index1, tmp2);
         editorGrid.getChildren().set(index2, tmp1);
      }

      @Override public GridPane getEditor()
      {
         return container;
      }

      @Override public T getValue()
      {
         return originalValue;
      }

      @Override public void setValue(T value)
      {
         int index = IterableEditor.this.value.indexOf(originalValue);
         if (index == -1)
            return;
         IterableEditor.this.value.set(index, value);
         originalValue = value;
         valueProperty.setValue(Collections.unmodifiableCollection(IterableEditor.this.value));
      }

   }

   private static class DefaultEditor<T> extends AbstractPropertyEditor<T, TextField>
   {

      DefaultEditor(Item property)
      {
         super(property, new TextField(), true);
         getEditor().setDisable(true);
         //noinspection unchecked
         setValue((T)property.getValue());
      }

      @Override protected ObservableValue<T> getObservableValue()
      {
         //noinspection unchecked
         return (ObservableValue<T>) getEditor().textProperty();
      }

      @Override public void setValue(T value)
      {
         getEditor().textProperty().setValue(Objects.toString(value, ""));
      }
   }

   private class ItemProperty implements Item
   {
      final Supplier<T> itemSupplier;
      final Consumer<T> itemConsumer;

      private ItemProperty(Supplier<T> itemSupplier, Consumer<T> itemConsumer)
      {
         this.itemSupplier = itemSupplier;
         this.itemConsumer = itemConsumer;
      }

      @Override public Class<?> getType()
      {
         return itemSupplier.get().getClass();
      }

      @Override public String getCategory()
      {
         return "Basic";
      }

      @Override public String getName()
      {
         return "Item";
      }

      @Override public String getDescription()
      {
         return "Item";
      }

      @Override public Object getValue()
      {
         return itemSupplier.get();
      }

      @Override public void setValue(Object value)
      {
         //noinspection unchecked
         itemConsumer.accept((T)value);
      }

      @Override public Optional<ObservableValue<?>> getObservableValue()
      {
         return Optional.empty();
      }
   }
}
