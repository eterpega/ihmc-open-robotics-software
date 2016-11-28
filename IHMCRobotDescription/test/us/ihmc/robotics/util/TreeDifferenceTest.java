package us.ihmc.robotics.util;

import org.junit.Test;
import us.ihmc.robotics.util.TreeDifference.Difference;
import us.ihmc.robotics.util.TreeDifference.DifferencesByNode;

import java.util.Optional;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static us.ihmc.robotics.util.TreeDifference.difference;

public class TreeDifferenceTest extends TreeTestBase
{
   @Test
   public void testDifferenceBetweenTheSameTreeIsEmpty()
   {
      assertTrue(difference(SINGLETON_TREE, SINGLETON_TREE).getAllDifferences().isEmpty());
      assertTrue(difference(BINARY_TREE, BINARY_TREE).getAllDifferences().isEmpty());
   }

   @Test
   public void testDifferenceWithAddedChildren()
   {
      DifferencesByNode<Integer> differences = difference(SINGLETON_TREE, BINARY_TREE);
      Optional<Difference<Integer>> diffOpt = differences.get(SINGLETON_TREE);
      assertTrue(diffOpt.isPresent());
      diffOpt.ifPresent(diff -> {
         assertEquals(2, diff.getAddedChildren().size());
         assertTrue(diff.getAddedChildren().containsAll(BINARY_TREE.getChildren()));
         assertEquals(BINARY_TREE, diff.getNewNode());
         assertTrue(diff.getRemovedChildren().isEmpty());
         assertEquals(0, (int)diff.getFinalChildOrder().apply(BINARY_TREE.getChild(0)));
         assertEquals(1, (int)diff.getFinalChildOrder().apply(BINARY_TREE.getChild(1)));
      });
   }

   @Test
   public void testDifferenceWithRemovedChildren()
   {
      DifferencesByNode<Integer> differences = difference(BINARY_TREE, SINGLETON_TREE);
      Optional<Difference<Integer>> diffOpt = differences.get(BINARY_TREE);
      assertTrue(diffOpt.isPresent());
      diffOpt.ifPresent(diff -> {
         assertEquals(2, diff.getRemovedChildren().size());
         assertTrue(diff.getRemovedChildren().containsAll(BINARY_TREE.getChildren()));
         assertEquals(SINGLETON_TREE, diff.getNewNode());
         assertTrue(diff.getAddedChildren().isEmpty());
         assertTrue(diff.getFinalChildOrder().isEmpty());
      });
   }

   @Test
   public void testDifferenceWithChangedRootValue()
   {
      DifferencesByNode<Integer> differences = difference(BINARY_TREE, BINARY_TREE.withValue(42));
      Optional<Difference<Integer>> diffOpt = differences.get(BINARY_TREE);
      assertTrue(diffOpt.isPresent());
      diffOpt.ifPresent(diff -> {
         assertEquals(42, (int)diff.getNewNode().getValue());
         assertTrue(diff.getRemovedChildren().isEmpty());
         assertTrue(diff.getAddedChildren().isEmpty());
         assertEquals(0, (int)diff.getFinalChildOrder().apply(BINARY_TREE.getChild(0)));
         assertEquals(1, (int)diff.getFinalChildOrder().apply(BINARY_TREE.getChild(1)));
      });
   }

   @Test
   public void testDifferenceWithSingleLeafReplaced()
   {
      Tree<Integer> tree1 = tree(1, tree(2), tree(3));
      Tree<Integer> tree2 = tree(1, tree1.getChild(0), tree(4));
      DifferencesByNode<Integer> differences = difference(tree1, tree2);
      Optional<Difference<Integer>> diffOpt = differences.get(tree1);
      assertTrue(diffOpt.isPresent());
      diffOpt.ifPresent(diff -> {
         assertTrue(diff.getRemovedChildren().isEmpty());
         assertTrue(diff.getAddedChildren().isEmpty());
         assertEquals(0, (int)diff.getFinalChildOrder().apply(tree2.getChild(0)));
         assertEquals(1, (int)diff.getFinalChildOrder().apply(tree2.getChild(1)));
      });

      diffOpt = differences.get(tree1.getChild(1));
      assertTrue(diffOpt.isPresent());
      diffOpt.ifPresent(diff -> {
         assertTrue(tree2.getChild(1).deepEquals(diff.getNewNode()));
         assertTrue(diff.getRemovedChildren().isEmpty());
         assertTrue(diff.getAddedChildren().isEmpty());
      });

      assertFalse(differences.get(tree1.getChild(0)).isPresent());
   }

   @Test
   public void testDifferenceWithSubtreeReplacementAndAddition()
   {
      Tree<Integer> movedAndChangedSubtree, unchangedSubtree;
      Tree<Integer> tree1 = tree(1,
                               tree(2,
                                    tree(3),
                                    unchangedSubtree = tree(4,
                                         tree(5),
                                         tree(6)),
                                    movedAndChangedSubtree = tree(7,
                                         tree(8),
                                         tree(9)),
                                    tree(10)
                                    ));

      Tree<Integer> tree2 = tree(1,
                                 tree(2,
                                      tree1.getChild(0).getChild(0),
                                      tree(11,
                                           tree1.getChild(0).getChild(2).getChild(0),
                                           tree1.getChild(0).getChild(2).getChild(1)),
                                      unchangedSubtree,
                                      tree1.getChild(0).getChild(3),
                                      tree(12)
                                 ));

      DifferencesByNode<Integer> differences = difference(tree1, tree2);

      // Root node was changed because children were changed
      assertTrue(differences.get(tree1).isPresent());

      // First child was changed - subtree was moved and updated and another leaf (12) was added
      Optional<Difference<Integer>> diffOpt = differences.get(tree1.getChild(0));
      assertTrue(diffOpt.isPresent());
      Difference<Integer> diff = diffOpt.get();

      assertTrue(diff.getAddedChildren().map(Tree::getValue).contains(12));
      assertTrue(diff.getRemovedChildren().isEmpty());

      // Check the order of the modified tree children
      Tree<Integer> newNode = tree2.getChild(0);
      assertEquals(0, (int)diff.getFinalChildOrder().apply(newNode.getChild(0)));
      //noinspection OptionalGetWithoutIsPresent
      assertEquals(1, (int)diff.getFinalChildOrder().apply(tree2.findValue(11).get().getFocusedNode()));
      //noinspection OptionalGetWithoutIsPresent
      assertEquals(2, (int)diff.getFinalChildOrder().apply(tree2.findValue(4).get().getFocusedNode()));
      assertEquals(3, (int)diff.getFinalChildOrder().apply(newNode.getChild(3)));
      assertEquals(4, (int)diff.getFinalChildOrder().apply(newNode.getChild(4)));

      // Check that the moved subtree was changed
      Optional<Difference<Integer>> movedChangedDiffOpt = differences.get(movedAndChangedSubtree);
      assertTrue(movedChangedDiffOpt.isPresent());
      assertEquals(11, (int)movedChangedDiffOpt.get().getNewNode().getValue());
      assertTrue(movedChangedDiffOpt.get().getAddedChildren().isEmpty());
      assertTrue(movedChangedDiffOpt.get().getRemovedChildren().isEmpty());


      // Check no extra changes are reported
      assertFalse(differences.get(tree1.getChild(0).getChild(0)).isPresent());
      assertFalse(differences.get(tree1.getChild(0).getChild(2).getChild(0)).isPresent());
      assertFalse(differences.get(tree1.getChild(0).getChild(3)).isPresent());
      assertFalse(differences.get(unchangedSubtree).isPresent());
   }

   @Test
   public void testChildrenSwappedAreProperlyRecognizedInTheDiff()
   {
      Tree<Integer> tree1 = tree(1, tree(2), tree(3));
      Tree<Integer> tree2 = tree(1, tree1.getChild(1), tree1.getChild(0));
      DifferencesByNode<Integer> diff = TreeDifference.difference(tree1, tree2);
      Optional<Difference<Integer>> rootDiffOpt = diff.get(tree1);
      assertTrue(rootDiffOpt.isPresent());
      assertEquals(0, rootDiffOpt.get().getAddedChildren().size());
      assertEquals(0, rootDiffOpt.get().getRemovedChildren().size());
      assertEquals(1, (int)rootDiffOpt.get().getFinalChildOrder().apply(tree1.getChild(0)));
      assertEquals(0, (int)rootDiffOpt.get().getFinalChildOrder().apply(tree1.getChild(1)));
   }
}
