package us.ihmc.robotbuilder.gui.editors;

import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.scene.Node;
import javafx.scene.control.Button;
import javafx.scene.control.TextField;
import javafx.scene.control.TitledPane;
import javafx.scene.layout.GridPane;
import javaslang.collection.Stream;
import us.ihmc.robotbuilder.gui.Editor;

import java.util.ArrayList;
import java.util.Collections;

import static us.ihmc.robotbuilder.util.FunctionalObservableValue.functional;
import static us.ihmc.robotbuilder.util.NoCycleProperty.noCycle;

/**
 *
 */
public class IterableEditor<T> extends Editor<Iterable<T>>
{
   private final GridPane editorGrid = new GridPane();
   private final Editor.Factory editorFactory;
   private final TitledPane editorContainer = new TitledPane();
   private ArrayList<T> value = new ArrayList<>();

   public IterableEditor(Property<Iterable<T>> valueProperty, Editor.Factory editorFactory)
   {
      super(noCycle(valueProperty));
      this.editorFactory = editorFactory;

      editorContainer.setExpanded(false);
      editorContainer.setText("Click to expand");
      editorContainer.setAnimated(false);

      editorContainer.expandedProperty().addListener(new ChangeListener<Boolean>()
      {
         @Override public void changed(ObservableValue<? extends Boolean> observable, Boolean oldValue, Boolean newValue)
         {
            // Create child items lazily
            if (!newValue)
               return;

            editorContainer.setContent(editorGrid);

            observeValueChanges();
            editorContainer.expandedProperty().removeListener(this);
         }
      });
   }

   private void observeValueChanges()
   {
      functional(valueProperty())
            .consume((newValue) ->
                     {
                        editorGrid.getChildren().clear();
                        value = new ArrayList<>();
                        for (T t : newValue)
                        {
                           value.add(t);
                        }

                        Stream.ofAll(value)
                              .zipWithIndex()
                              .map(listItemWithIndex -> new ListItemEditor((int) (long) listItemWithIndex._2))
                              .forEach(editor ->
                                       {
                                          GridPane.setRowIndex(editor, editor.itemIndex);
                                          editorGrid.getChildren().add(editor);
                                       });

                        Button addButton = new Button("Add");
                        GridPane.setRowIndex(addButton, editorGrid.getChildren().size());
                        editorGrid.getChildren().add(addButton);
                     });
   }

   @Override public Node getEditor()
   {
      return editorContainer;
   }

   private class ListItemEditor extends GridPane
   {
      private final Button remove = new Button("\uf00d");
      private final Button moveUp = new Button("\uf077");
      private final Button moveDown = new Button("\uf078");
      private Editor<T> innerEditor;
      private final Property<T> itemProperty;
      private int itemIndex;

      ListItemEditor(int itemIndex)
      {
         this.itemIndex = itemIndex;
         itemProperty = noCycle(new SimpleObjectProperty<T>(value.get(itemIndex)) {
            @Override public String getName()
            {
               return valueProperty().getName() + "[" + ListItemEditor.this.itemIndex + "]";
            }
         });

         itemProperty.addListener((observable, oldValue, newValue) ->
                                  {
                                     IterableEditor.this.value.set(itemIndex, newValue);
                                     valueProperty().setValue(Collections.unmodifiableCollection(IterableEditor.this.value));
                                  });

         //noinspection unchecked
         innerEditor = (Editor<T>)editorFactory.create(itemProperty.getValue().getClass(), itemProperty)
                                    .orElse(new DefaultEditor<>(itemProperty));

         Node[] nodes = {remove, moveUp, moveDown, innerEditor.getEditor()};
         for (int i = 0; i < nodes.length; i++)
         {
            GridPane.setColumnIndex(nodes[i], i);
         }
         getChildren().addAll(nodes);

         remove.setOnAction(event ->
                            {
                               editorGrid.getChildren().remove(this);
                               //noinspection unchecked
                               Stream.of(editorGrid.getChildren())
                                                          .map(x -> (ListItemEditor) x)
                                                          .zipWithIndex()
                                                          .forEach(editorItem -> editorItem._1.itemIndex = (int)(long)editorItem._2);
                               value.remove(itemProperty.getValue());
                               updateValue();
                            });

         moveUp.setOnAction(event -> {
            int index = value.indexOf(itemProperty.getValue());
            if (index > 0)
            {
               value.set(index, value.get(index - 1));
               value.set(index - 1, itemProperty.getValue());
               swapEditors(index - 1, index);
               updateValue();
            }
         });

         moveDown.setOnAction(event -> {
            int index = value.indexOf(itemProperty.getValue());
            if (index >= 0 && index < value.size() - 1)
            {
               value.set(index, value.get(index + 1));
               value.set(index + 1, itemProperty.getValue());
               swapEditors(index, index + 1);
               updateValue();
            }
         });

         Stream.of(moveUp, moveDown, remove).forEach(button -> button.setStyle("-fx-font-family: \"FontAwesome\";"));
      }

      private void updateValue()
      {
         valueProperty().setValue(Collections.unmodifiableCollection(IterableEditor.this.value));
      }

      private void swapEditors(int index1, int index2)
      {
         @SuppressWarnings("unchecked") ListItemEditor editor1 = (ListItemEditor) editorGrid.getChildren().get(index1);
         @SuppressWarnings("unchecked") ListItemEditor editor2 = (ListItemEditor) editorGrid.getChildren().get(index2);
         GridPane.setRowIndex(editor1, index2);
         GridPane.setRowIndex(editor2, index1);
         editorGrid.getChildren().set(index2, new GridPane());
         editorGrid.getChildren().set(index1, editor2);
         editorGrid.getChildren().set(index2, editor1);
         editor1.itemIndex = index2;
         editor2.itemIndex = index1;
      }
   }

   private static class DefaultEditor<T> extends Editor<T>
   {
      private final TextField editor = new TextField();

      DefaultEditor(Property<T> property)
      {
         super(property);
         getEditor().setDisable(true);
      }

      @Override public Node getEditor()
      {
         return editor;
      }
   }
}
