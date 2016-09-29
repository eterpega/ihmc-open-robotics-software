package us.ihmc.robotbuilder.util;

import javafx.scene.control.TreeItem;
import javaslang.collection.List;
import org.junit.Test;
import us.ihmc.tools.testing.JUnitTools;

import java.util.Iterator;
import java.util.Optional;
import java.util.stream.Collectors;

import static javaslang.collection.List.empty;
import static org.junit.Assert.*;

/**
 *
 */
public class TreeTest
{
   private static final Tree<Integer> BINARY_TREE = new Tree<>(0, List.of(new Tree<>(1, empty()), new Tree<>(2, empty())));
   private static final Tree<Integer> SINGLETON_TREE = new Tree<>(0, empty());

   @Test
   public void testEqualsHashCode()
   {
      JUnitTools.testHashCodeEqualsMethods(BINARY_TREE, BINARY_TREE.mapValues(x -> x), SINGLETON_TREE);
   }

   @Test
   public void testNodeDoesNotEqualNullOrAnotherClass()
   {
      //noinspection ObjectEqualsNull
      assertFalse(BINARY_TREE.equals(null));
      //noinspection EqualsBetweenInconvertibleTypes
      assertFalse(BINARY_TREE.equals("nonsense"));
   }

   @Test
   public void testToStringContainsValue()
   {
      Tree<String> test = new Tree<>("abcd1234", empty());
      assertTrue(test.toString().contains(test.getValue()));
   }

   @Test
   public void testAdaptCoversTheWholeTree()
   {
      TreeItem<String> test = new TreeItem<>("0");
      test.getChildren().add(new TreeItem<>("1"));
      test.getChildren().add(new TreeItem<>("2"));

      Tree<TreeItem<String>> adaptedTree = Tree.adapt(test, TreeItem::getChildren);
      String treeStr = adaptedTree.stream().map(TreeItem::getValue).collect(Collectors.joining());
      assertEquals(treeStr, "012");
   }

   @Test
   public void testTreeStoresChildrenInOrder()
   {
      Iterator<Tree<Integer>> iter = BINARY_TREE.getChildren().iterator();
      assertEquals((int)iter.next().getValue(), 1);
      assertEquals((int)iter.next().getValue(), 2);
   }

   @Test
   public void testMapMapsToAnUnrelatedTree()
   {
      TreeItem<String> mapped = BINARY_TREE.map((node, children) ->
      {
         TreeItem<String> result = new TreeItem<>(Integer.toString(node.getValue()));
         result.getChildren().addAll(children);
         return result;
      });

      assertEquals(mapped.getValue(), "0");
      assertEquals(mapped.getChildren().get(0).getValue(), "1");
      assertEquals(mapped.getChildren().get(1).getValue(), "2");
   }

   @Test
   public void testValueMapMapsAllValues()
   {
      Tree<String> mapped = BINARY_TREE.mapValues(x -> Integer.toString(x));
      Iterator<Tree<String>> iter = mapped.getChildren().iterator();
      assertEquals(mapped.getValue(), "0");
      assertEquals(iter.next().getValue(), "1");
      assertEquals(iter.next().getValue(), "2");
   }

   @Test
   public void testFalsePredicateFilterFiltersAll()
   {
      assertFalse(BINARY_TREE.filter(x -> false).isPresent());
      assertFalse(BINARY_TREE.filterValues(x -> false).isPresent());
   }

   @Test
   public void testTruePredicateKeepsTreeUntouched()
   {
      assertEquals(BINARY_TREE.filter(x -> true), Optional.of(BINARY_TREE));
      assertEquals(BINARY_TREE.filterValues(x -> true), Optional.of(BINARY_TREE));
   }

   @Test
   public void testFilterJustSomeNodes()
   {
      assertEquals(2L, (long)BINARY_TREE.filterValues(x -> x < 2).map(tree -> tree.stream().count()).orElse(-1L));
   }

   @Test
   public void testStreamReturnsAllValues()
   {
      assertEquals(3, (int)BINARY_TREE.stream().reduce(0, (x,  y) -> x + y));
   }

   @Test
   public void testFocusCreatesARootFocus()
   {
      TreeFocus<Tree<Integer>> focus = BINARY_TREE.getFocus();
      assertEquals(focus.getFocusedNode(), BINARY_TREE);
      assertEquals(focus.root(), focus);
   }

   @Test
   public void testFindTrueOnSingletonTreeReturnsRoot()
   {
      Optional<TreeFocus<Tree<Integer>>> findResult = SINGLETON_TREE.find(x -> true);
      assertTrue(findResult.isPresent());
      findResult.ifPresent(focus -> assertEquals(focus.getFocusedNode(), SINGLETON_TREE));
   }

   @Test
   public void testFindFalseReturnsNothing()
   {
      assertFalse(BINARY_TREE.find(x -> false).isPresent());
   }

   @Test
   public void testFindChildNodeReturnsTheCorrectNode()
   {
      Optional<TreeFocus<Tree<Integer>>> findResult = BINARY_TREE.find(node -> node.getValue() == 1);
      assertNodeFound(findResult);
   }

   @Test
   public void testFindChildValueReturnsTheCorrectNode()
   {
      Optional<TreeFocus<Tree<Integer>>> findResult = BINARY_TREE.findValue(value -> value == 1);
      assertNodeFound(findResult);
   }

   @SuppressWarnings("OptionalUsedAsFieldOrParameterType")
   private static void assertNodeFound(Optional<TreeFocus<Tree<Integer>>> findResult)
   {
      findResult.map(foundNode -> {
         assertEquals(foundNode.getFocusedNode(), BINARY_TREE.getChildren().iterator().next());
         return foundNode;
      }).orElseGet(() -> {
         assertTrue("Node not found", false);
         return null;
      });
   }
}
