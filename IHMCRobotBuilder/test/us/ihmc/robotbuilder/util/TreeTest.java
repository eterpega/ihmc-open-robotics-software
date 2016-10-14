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

public class TreeTest extends TreeTestBase
{
   @Test public void testFirstChildReturnsFirstChildOfNonEmptyTree()
   {
      assertEquals(BINARY_TREE.firstChild(), Optional.of(BINARY_TREE.getChildren().iterator().next()));
   }

   @Test public void testFirstChildReturnsEmptyForEmptyTree()
   {
      assertFalse(SINGLETON_TREE.firstChild().isPresent());
   }

   @Test public void testGetChildReturnsTheCorrectChild()
   {
      List<Tree<Integer>> children = BINARY_TREE.childStream().collect(List.collector());
      assertEquals(BINARY_TREE.getChild(0), children.get(0));
      assertEquals(BINARY_TREE.getChild(1), children.get(1));

      try {
         BINARY_TREE.getChild(BINARY_TREE.countChildren());
         assertTrue("No IndexOutOfBoundsException thrown", false);
      } catch (Exception ex) {
         assertTrue(ex instanceof IndexOutOfBoundsException);
      }
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
      assertTrue(BINARY_TREE.filter(x -> true).map(BINARY_TREE::deepEquals).orElse(false));
      assertTrue(BINARY_TREE.filterValues(x -> true).map(BINARY_TREE::deepEquals).orElse(false));
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
   public void testFindChildValueBasedOnPredicateReturnsTheCorrectNode()
   {
      Optional<TreeFocus<Tree<Integer>>> findResult = BINARY_TREE.findValue(value -> value == 1);
      assertNodeFound(findResult);
   }

   @Test
   public void testFindChildValueReturnsTheCorrectNode()
   {
      Optional<TreeFocus<Tree<Integer>>> findResult = BINARY_TREE.findValue(1);
      assertNodeFound(findResult);
   }

   @Test
   public void testWithValueReturnsANewNodeWithTheAppropriateValue()
   {
      Tree<Integer> newTree = BINARY_TREE.withValue(42);
      assertEquals(42, (int)newTree.getValue());
      assertEquals(BINARY_TREE.getChild(0), newTree.getChild(0));
      assertEquals(BINARY_TREE.getChild(1), newTree.getChild(1));
   }

   @Test
   public void testDeepEqualsHashCode()
   {
      class Wrapper {
         private final Tree<Integer> tree;

         private Wrapper(Tree<Integer> tree)
         {
            this.tree = tree;
         }

         @Override public boolean equals(Object o)
         {
            if (this == o)
               return true;
            if (o == null || getClass() != o.getClass())
               return false;

            Wrapper wrapper = (Wrapper) o;

            return tree.deepEquals(wrapper.tree);

         }

         @Override public int hashCode()
         {
            return tree.deepHashCode();
         }
      }
      JUnitTools.testHashCodeEqualsMethods(new Wrapper(DEEP_TREE), new Wrapper(DEEP_TREE.mapValues(x -> x)), new Wrapper(BINARY_TREE));
   }

   @Test
   public void testDrawSingletonTreeEqualsValueToString()
   {
      assertEquals(SINGLETON_TREE.getValue().toString(), SINGLETON_TREE.draw());
   }

   @Test
   public void testDrawSingletonTreeContainsAllValues()
   {
      String drawing = BINARY_TREE.draw();
      assertTrue(BINARY_TREE.children().map(Tree::getValue).map(Object::toString).forAll(drawing::contains));
   }

   @Test
   public void testDrawTreeAgainstKnownResult()
   {
      Tree<Integer> treeToShow = tree(1, tree(2, tree(2)), tree(4));
      assertEquals("1\n├──2\n│  └──2\n└──4", treeToShow.draw());
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
