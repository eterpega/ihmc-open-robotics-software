package us.ihmc.robotbuilder.gui.editors;

import impl.org.controlsfx.skin.BreadCrumbBarSkin.BreadCrumbButton;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ObservableValue;
import javafx.geometry.Insets;
import javafx.scene.Node;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.control.TreeItem;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.FlowPane;
import javaslang.Lazy;
import javaslang.Tuple;
import javaslang.Tuple2;
import org.controlsfx.control.BreadCrumbBar;
import us.ihmc.robotbuilder.gui.Editor;
import us.ihmc.robotbuilder.gui.FontAwesomeLabel;
import us.ihmc.robotics.immutableRobotDescription.NamedObject;

import java.util.*;

import static us.ihmc.robotics.util.FunctionalObservableValue.functional;
import static us.ihmc.robotics.util.NoCycleProperty.noCycle;

/**
 *
 */
public class RecursiveBeanEditor<T> extends Editor<T>
{

   private final BorderPane container = new BorderPane();
   private final Stack<Tuple2<ObservableValue<String>, ImmutableEditor<?>>> editorStack = new Stack<>();
   private final BreadCrumbBar<BreadCrumbItem> breadCrumbBar = new BreadCrumbBar<>();

   public RecursiveBeanEditor(Property<T> valueProperty)
   {
      super(noCycle(valueProperty));
      Property<T> rootEditorProperty = noCycle(new SimpleObjectProperty<>());
      ImmutableEditor<T> rootEditor = new ImmutableEditor<>(rootEditorProperty, new RecursiveEditorFactory());
      functional(valueProperty())
            .consume(objectToEdit -> {
               if (objectToEdit == rootEditorProperty.getValue())
               {
                  return;
               }
               rootEditorProperty.setValue(objectToEdit);
               editorStack.clear();
               editorStack.push(Tuple.of(functional(valueProperty()).map(RecursiveBeanEditor::objectName), rootEditor));
               container.setTop(breadCrumbBar);
               updateEditorState();
            });

      rootEditorProperty.addListener((observable, oldValue, newValue) -> valueProperty().setValue(newValue));

      setupBreadCrumbBar();
   }

   private void setupBreadCrumbBar()
   {
      breadCrumbBar.setOnCrumbAction(event -> revertTo(event.getSelectedCrumb().getValue().index));

      breadCrumbBar.setCrumbFactory(crumb -> {
         BreadCrumbButton button = new BreadCrumbButton(crumb.getValue().name.getValue());
         button.textProperty().bind(crumb.getValue().name);
         return button;
      });
   }

   private static String objectName(Object object)
   {
      if (object instanceof NamedObject)
         return ((NamedObject) object).getName();
      return Objects.toString(object);
   }

   private void updateEditorState()
   {
      if (editorStack.size() > 0)
      {
         container.setCenter(editorStack.peek()._2.getEditor());
         BorderPane.setMargin(container.getCenter(), new Insets(5, 0, 0, 10));
      }
      else
         container.setCenter(null);

      updateBreadcrumbs();
   }

   private void updateBreadcrumbs()
   {
      TreeItem<BreadCrumbItem> item = null;
      for (int i = 0; i < editorStack.size(); i++)
      {
         TreeItem<BreadCrumbItem> lastItem = item;
         item = new TreeItem<>(new BreadCrumbItem(i, editorStack.get(i)._1));
         if (lastItem != null)
            lastItem.getChildren().add(item);
      }

      breadCrumbBar.setSelectedCrumb(item);
   }

   private void revertTo(int index)
   {
      if (index < 0)
         return;

      while (editorStack.size() > index + 1)
         editorStack.pop();
      updateEditorState();
   }

   @Override public Node getEditor()
   {
      return container;
   }

   private class RecursiveEditorFactory extends PropertyEditorFactory
   {
      @Override public Optional<Editor<?>> create(Class<?> clazz, List<Class<?>> genericParameters, Property<?> value)
      {
         return Optional.of(super.create(clazz, genericParameters, value).orElse(new InnerBeanEditor<>(value, this)));
      }
   }

   private class InnerBeanEditor<U> extends Editor<U>
   {
      private final Lazy<ImmutableEditor<U>> innerEditor;
      private final FlowPane container = new FlowPane();

      InnerBeanEditor(Property<U> property, Factory factory)
      {
         super(property);
         container.setHgap(10);
         innerEditor = Lazy.of(() -> new ImmutableEditor<>(property, factory));
         Button edit = new Button("Edit...", new FontAwesomeLabel("\uf040"));
         edit.setOnAction(e ->
                          {
                             editorStack.push(Tuple.of(new SimpleObjectProperty<>(property.getName()), innerEditor.get()));
                             updateEditorState();
                          });
         U value = property.getValue();
         if (value != null)
         {
            String text = objectName(value);
            if (text.length() > 25)
               text = text.substring(0, 25) + "...";
            Label textLabel = new Label(text);
            container.getChildren().add(textLabel);
         }
         container.getChildren().add(edit);
      }

      @Override public Node getEditor()
      {
         return container;
      }
   }

   private static class BreadCrumbItem
   {
      private final int index;
      private final ObservableValue<String> name;

      private BreadCrumbItem(int index, ObservableValue<String> name)
      {
         this.index = index;
         this.name = name;
      }
   }
}
