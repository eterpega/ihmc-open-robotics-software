package us.ihmc.robotbuilder.util;

import javafx.scene.control.TreeItem;
import org.junit.Test;

import java.util.List;

import static org.junit.Assert.*;

public class TreeMappingTest extends TreeTestBase
{
   @Test
   public void testMapNewNodeRetainsTheMappedNode()
   {
      TreeMapping<Tree<Integer>, TreeItem<Integer>> mapping = new TreeMapping<>(TreeMappingTest::mapToTreeItem);
      Tree<Integer> base = tree(1);
      TreeItem<Integer> mapped = mapping.mapNewNode(base);
      assertEquals(1, (int)mapped.getValue());
      assertEquals(0, mapped.getChildren().size());

      //noinspection OptionalGetWithoutIsPresent
      assertTrue(mapping.get(base).get() == mapped);
   }

   @Test
   public void testRemoveMappingDiscardsExistingMapping()
   {
      TreeMapping<Tree<Integer>, TreeItem<Integer>> mapping = new TreeMapping<>(TreeMappingTest::mapToTreeItem);
      Tree<Integer> base = tree(1);
      mapping.mapNewNode(base);

      mapping.removeMapping(base);

      assertFalse(mapping.get(base).isPresent());
   }

   @SuppressWarnings("OptionalGetWithoutIsPresent") @Test
   public void testReplaceSingleNodeOnlyReplacesOneNode()
   {
      TreeMapping<Tree<Integer>, TreeItem<Integer>> mapping = new TreeMapping<>(TreeMappingTest::mapToTreeItem);
      Tree<Integer> baseReplaced = tree(42);
      TreeItem<Integer> mapped = mapping.mapNewNode(BINARY_TREE);
      mapping.replaceSingleNode(BINARY_TREE, baseReplaced);
      assertFalse(mapping.get(BINARY_TREE).isPresent());
      assertEquals(mapped, mapping.get(baseReplaced).get());
      for (Tree<Integer> child : BINARY_TREE.getChildren())
      {
         assertEquals(child.getValue(), mapping.get(child).get().getValue());
      }
   }

   @Test
   public void testGetReverseReturnsPreviouslyMappedNode()
   {
      TreeMapping<Tree<Integer>, TreeItem<Integer>> mapping = new TreeMapping<>(TreeMappingTest::mapToTreeItem);
      Tree<Integer> base = tree(42);
      TreeItem<Integer> mapped = mapping.mapNewNode(base);
      //noinspection OptionalGetWithoutIsPresent
      assertEquals(base, mapping.getReverse(mapped).get());
   }

   @Test
   public void testGetAndGetReverseReturnEmptyForNonMappedNodes()
   {
      TreeMapping<Tree<Integer>, TreeItem<Integer>> mapping = new TreeMapping<>(TreeMappingTest::mapToTreeItem);
      assertFalse(mapping.get(BINARY_TREE).isPresent());
      assertFalse(mapping.getReverse(new TreeItem<>()).isPresent());

      mapping.mapNewNode(tree(42));

      // Re-check with non-empty mapping
      assertFalse(mapping.get(BINARY_TREE).isPresent());
      assertFalse(mapping.getReverse(new TreeItem<>()).isPresent());
   }

   private static <T> TreeItem<T> mapToTreeItem(Tree<T> tree, List<TreeItem<T>> children)
   {
      TreeItem<T> result = new TreeItem<>(tree.getValue());
      result.getChildren().addAll(children);
      return result;
   }
}
