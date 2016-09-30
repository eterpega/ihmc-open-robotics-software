package us.ihmc.robotbuilder.gui.editors;

import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.scene.Node;
import javafx.scene.control.Button;
import javafx.scene.control.TextField;
import javafx.scene.control.TitledPane;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.GridPane;
import org.controlsfx.control.PropertySheet.Item;
import org.controlsfx.property.editor.AbstractPropertyEditor;
import org.controlsfx.property.editor.PropertyEditor;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.stream.Collectors;

/**
 *
 */
public class IterableEditor<T> implements PropertyEditor<Iterable<T>>
{
   private final TitledPane editorContainer = new TitledPane();
   private final GridPane editorGrid = new GridPane();
   private final EditorFactory editorFactory;
   private List<T> value = new ArrayList<>();

   public IterableEditor(EditorFactory editorFactory)
   {
      this.editorFactory = editorFactory;

      editorContainer.setExpanded(false);
      editorContainer.setText("Click to expand");
      editorContainer.setContent(editorGrid);
      editorContainer.setAnimated(false);
   }

   @Override public Node getEditor()
   {
      return editorContainer;
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
      if (this.value.isEmpty())
         return;

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

   }

   private class ListItemEditor implements PropertyEditor<T>
   {
      private final Button remove = new Button("Remove");
      private final Button moveUp = new Button("Move Up");
      private final Button moveDown = new Button("Move Down");
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
                            });
      }

      @Override public Node getEditor()
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
