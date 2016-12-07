package us.ihmc.robotbuilder.gui.editors;

import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.scene.Node;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.control.TitledPane;
import javafx.scene.layout.GridPane;
import javaslang.collection.Stream;
import us.ihmc.robotbuilder.gui.Creator;
import us.ihmc.robotbuilder.gui.Editor;
import us.ihmc.robotbuilder.gui.FontAwesomeLabel;
import us.ihmc.robotics.immutableRobotDescription.NamedObject;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Objects;
import java.util.stream.Collectors;
import java.util.stream.StreamSupport;

import static us.ihmc.robotics.util.FunctionalObservableValue.functional;
import static us.ihmc.robotics.util.NoCycleProperty.noCycle;

/**
 *
 */
public class IterableEditor<T> extends Editor<Iterable<T>>
{
   private final GridPane editorGrid = new GridPane();
   private final Editor.Factory editorFactory;
   private final TitledPane editorContainer = new TitledPane();
   private ArrayList<T> value = new ArrayList<>();
   private final Class<T> itemType;

   public IterableEditor(Property<Iterable<T>> valueProperty, Class<T> itemType, Editor.Factory editorFactory)
   {
      super(noCycle(valueProperty));
      this.itemType = itemType;
      this.editorFactory = editorFactory;

      editorContainer.setExpanded(false);
      editorContainer.setText("[]");
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

      observeTitleChanges();
   }

   private void observeTitleChanges()
   {
      final int maxTextLength = 70;
      functional(valueProperty()).consume((newValue) ->
                                  {
                                     String text = StreamSupport.stream(newValue.spliterator(), false)
                                                                .map(item -> item instanceof NamedObject ? ((NamedObject)item).getName() : item.toString())
                                                                .collect(Collectors.joining(", "));
                                     if (text.length() > maxTextLength)
                                       text = text.substring(0, maxTextLength - 4) + " ...";
                                     if (text.isEmpty())
                                     {
                                        editorContainer.setText("[]");
                                     }
                                     else
                                     {
                                        editorContainer.setText(text);
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

                        addButton.setOnAction(event ->
                                              {
                                                 CreatorFactory factory = new CreatorFactory();
                                                 factory.create(itemType, Collections.emptyList())
                                                        .map(Creator::create)
                                                        .map(future -> future.onSuccess(newItemOpt -> newItemOpt.ifPresent(newItem ->
                                                          {
                                                             //noinspection unchecked
                                                             value.add(
                                                                   (T) newItem);
                                                             updateValue();
                                                          })
                                                 ));
                                              });
                     });
   }

   @Override public Node getEditor()
   {
      return editorContainer;
   }

   private void updateValue()
   {
      valueProperty().setValue(Collections.unmodifiableCollection(IterableEditor.this.value));
   }

   private class ListItemEditor extends GridPane
   {
      private final Button remove = new Button("", new FontAwesomeLabel("\uf00d"));
      private final Button moveUp = new Button("", new FontAwesomeLabel("\uf077"));
      private final Button moveDown = new Button("", new FontAwesomeLabel("\uf078"));
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
         innerEditor = (Editor<T>)editorFactory.create(itemProperty.getValue().getClass(), Collections.emptyList(), itemProperty)
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
      private final Label editor = new Label();

      DefaultEditor(Property<T> property)
      {
         super(property);
         editor.textProperty().bind(functional(property).map(Objects::toString));
      }

      @Override public Node getEditor()
      {
         return editor;
      }
   }
}
