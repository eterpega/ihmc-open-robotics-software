package us.ihmc.robotbuilder.gui;

import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.scene.control.Label;
import javafx.scene.control.TreeItem;
import javafx.scene.control.TreeView;

import java.util.List;
import java.util.Optional;

/**
 *
 */
public class ListEditor<T> extends Editor<List<T>>
{
   private final TreeView<Editor<T>> editorView = new TreeView<>();
   private final Property<List<T>> listProperty = new SimpleObjectProperty<>();
   private final TreeItem<Editor<T>> root;

   public ListEditor(String label, String help)
   {
      super(label, help);

      getChildren().add(editorView);

      //noinspection unchecked
      root = new TreeItem<>(new Root((Class<T>) Object.class));

      listProperty.addListener((observable, oldValue, newValue) ->
                               {
                                  root.getChildren().clear();
                                  newValue.stream()
                                          .map(item -> Editor.editorForObject(item, label, help))
                                          .filter(Optional::isPresent)
                                          .map(Optional::get)
                                          .map(TreeItem::new)
                                          .forEach(root.getChildren()::add);
                               });
   }

   @Override public Class<List<T>> getValueType()
   {
      //noinspection unchecked
      return (Class<List<T>>)(Class)List.class;
   }

   @Override public Property<List<T>> valueProperty()
   {
      return listProperty;
   }

   private class Root extends Editor<T> {
      private final Class<T> clazz;
      private Property<T> valueProperty = new SimpleObjectProperty<>();

      private Root(Class<T> clazz)
      {
         super(ListEditor.this.getLabel(), ListEditor.this.getHelp());
         this.clazz = clazz;

         getChildren().add(new Label(getLabel()));
      }

      @Override public Class<T> getValueType()
      {
         return clazz;
      }

      @Override public Property<T> valueProperty()
      {
         return valueProperty;
      }
   }
}
