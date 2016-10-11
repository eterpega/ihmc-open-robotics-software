package us.ihmc.robotbuilder.gui;

import javafx.scene.control.TreeCell;
import javafx.scene.control.TreeItem;
import javafx.scene.control.TreeView;
import us.ihmc.robotbuilder.util.FunctionalObservableValue;
import us.ihmc.robotbuilder.util.Tree;
import us.ihmc.robotbuilder.util.TreeDifference;
import us.ihmc.robotbuilder.util.TreeDifference.DifferencesByNode;
import us.ihmc.robotbuilder.util.TreeFocus;
import us.ihmc.robotbuilder.util.TreeFocus.ChildIndexOf;
import us.ihmc.robotics.immutableRobotDescription.JointDescription;

import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import static us.ihmc.robotbuilder.util.TreeFocus.pathTo;

/**
 * Represents a tree view of a robot tree (see {@link Tree<JointDescription>}).
 */
public class JointTreeView extends TreeView<JointDescription>
{
   public JointTreeView()
   {
      setCellFactory(JointTreeCell::new);
   }

   private JointTreeItem mapTree(Tree<JointDescription> tree)
   {
      return tree.map((node, children) ->
         {
            JointTreeItem item = new JointTreeItem(node);
            item.getChildren().addAll(children);
            item.setExpanded(true);
            return item;
         }
      );
   }

   private void applyDifferencesByNode(JointTreeItem parentItem, DifferencesByNode<JointDescription> differencesByNode)
   {
      differencesByNode.get(parentItem.getOriginalTree()).forEach(diff ->
         diff.match((valueChanged) -> parentItem.setOriginalTree(valueChanged.getNewNode()),
                    (childrenAdded) -> parentItem.getChildren().addAll(childrenAdded.getNewChildren().map(this::mapTree).toJavaList()),
                    (childrenRemoved) -> parentItem.getChildrenSpecific().removeIf(child -> childrenRemoved.getRemovedChildren().contains(child.getOriginalTree())),
                    (childOrderChanged) ->
                    {
                       Stream<Integer> newIndices = parentItem.getChildrenSpecific().stream()
                                                                         .map(JointTreeItem::getOriginalTree)
                                                                         .map(childOrderChanged.getChildIndices()::get)
                                                                         .map(optIndex -> optIndex.getOrElse(() -> { assert false; return 0; }));
                       List<JointTreeItem> newChildren = newIndices.map(parentItem.getChildrenSpecific()::get).collect(Collectors.toList());
                       parentItem.getChildren().clear();
                       parentItem.getChildren().addAll(newChildren);
                    })
      );

      parentItem.getChildrenSpecific().forEach(child -> applyDifferencesByNode(child, differencesByNode));
   }

   public FunctionalObservableValue<TreeFocus<Tree<JointDescription>>> selectedNodeObservable()
   {
      ChildIndexOf<TreeItem<JointDescription>> childIndexOf = (tree, child) -> tree.getChildren().indexOf(child);

      return FunctionalObservableValue.of(getSelectionModel().selectedItemProperty())
                                      .filter(item -> item instanceof JointTreeItem)
                                      .map(item -> (JointTreeItem)item)
                                      .flatMapOptional(selectedItem -> {
                                         JointTreeItem root = rootOf(selectedItem);
                                         TreeFocus<Tree<JointDescription>> rootFocus = root.getOriginalTree().getFocus();
                                         Optional<TreeFocus<Tree<JointDescription>>> result = rootFocus
                                               .focusOnPath(pathTo(selectedItem, childIndexOf, TreeItem::getParent));

                                         assert result.isPresent();
                                         return result;
                                      });
   }

   private static JointTreeItem rootOf(JointTreeItem item)
   {
      JointTreeItem result = item;
      while (result.getParent() instanceof JointTreeItem)
      {
         result = (JointTreeItem)result.getParent();
      }
      return result;
   }

   /**
    * Change the root joint of this tree view.
    * The new root will be diffed against the existing tree and
    * only necessary changes will be applied to the tree.
    * @param newRoot new root
    */
   public void setRootJoint(Tree<JointDescription> newRoot)
   {
      if (newRoot == null)
      {
         setRoot(null);
         return;
      }

      if (!(getRoot() instanceof JointTreeItem))
      {
         setRoot(mapTree(newRoot));
         return;
      }

      JointTreeItem itemRoot = (JointTreeItem) getRoot();
      applyDifferencesByNode(itemRoot, TreeDifference.difference(itemRoot.getOriginalTree(), newRoot));
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

   private static class JointTreeItem extends TreeItem<JointDescription>
   {
      private Tree<JointDescription> originalTree;

      JointTreeItem(Tree<JointDescription> originalTree)
      {
         super(originalTree.getValue());
         this.originalTree = originalTree;
      }

      Tree<JointDescription> getOriginalTree()
      {
         return originalTree;
      }

      void setOriginalTree(Tree<JointDescription> originalTree)
      {
         this.originalTree = originalTree;
         this.setValue(originalTree.getValue());
      }

      List<JointTreeItem> getChildrenSpecific()
      {
         return getChildren().stream().map(x -> (JointTreeItem)x).collect(Collectors.toList());
      }
   }
}
