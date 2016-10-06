package us.ihmc.robotbuilder.gui;

import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.scene.control.TreeCell;
import javafx.scene.control.TreeItem;
import javafx.scene.control.TreeView;
import us.ihmc.robotbuilder.util.FunctionalObservableValue;
import us.ihmc.robotbuilder.util.Tree;
import us.ihmc.robotbuilder.util.TreeFocus;
import us.ihmc.robotbuilder.util.TreeInterface.CachedMapper;
import us.ihmc.robotics.immutableRobotDescription.JointDescription;

import java.util.Optional;
import java.util.function.Function;

import static us.ihmc.robotbuilder.util.TreeInterface.cachedMap;

/**
 *
 */
public class JointTreeView extends TreeView<JointDescription>
{
   private final Property<Optional<TreeFocus<Tree<JointDescription>>>> jointTree = new SimpleObjectProperty<>(Optional.empty());

   private final CachedMapper<Tree<JointDescription>, TreeItem<JointDescription>> treeMapper;

   public JointTreeView()
   {
      setCellFactory(JointTreeCell::new);

      treeMapper = cachedMap((node, children) -> {
         TreeItem<JointDescription> item = new TreeItem<>(node.getValue());
         item.getChildren().addAll(children);
         item.setExpanded(true);
         return item;
      });

      FunctionalObservableValue.of(jointTree)
            .flatMapOptional(Function.identity())
            .consume(focus -> {
               TreeItem<JointDescription> mappedTree = treeMapper.map(focus.root().getFocusedNode());
               setRoot(mappedTree);
            });
   }

   public Property<Optional<TreeFocus<Tree<JointDescription>>>> jointTreeProperty()
   {
      return jointTree;
   }

   public void setRootJoint(Tree<JointDescription> root)
   {
      jointTree.setValue(Optional.of(root.getFocus()));
   }

   private static class JointTreeCell extends TreeCell<JointDescription>
   {
      JointTreeCell(TreeView<JointDescription> tree) {
      }

      @Override protected void updateItem(JointDescription item, boolean empty)
      {
         super.updateItem(item, empty);

         if (empty)
         {
            setText(null);
         }
         else
         {
            setText(getItem() == null ? "" : getItem().getName());
         }
         setGraphic(null);
      }
   }
}
