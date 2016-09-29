package us.ihmc.robotbuilder.util;

import javaslang.collection.List;
import org.junit.Test;
import us.ihmc.robotbuilder.util.TreeFocus.Breadcrumb;
import us.ihmc.tools.testing.JUnitTools;

import java.util.Optional;

import static javaslang.collection.List.empty;
import static org.junit.Assert.*;

/**
 *
 */
public class TreeFocusTest
{
   private static final Tree<Integer> BINARY_TREE = new Tree<>(0, List.of(new Tree<>(1, empty()), new Tree<>(2, empty())));
   private static final Tree<Integer> SINGLETON_TREE = new Tree<>(0, empty());

   @Test
   public void testEqualsHashCode()
   {
      //noinspection OptionalGetWithoutIsPresent
      JUnitTools.testHashCodeEqualsMethods(BINARY_TREE.getFocus(), BINARY_TREE.getFocus(), BINARY_TREE.getFocus().firstChild().get());
      JUnitTools.testHashCodeEqualsMethods(new Breadcrumb<>(BINARY_TREE, empty(), empty()),
                                           new Breadcrumb<>(BINARY_TREE, empty(), empty()),
                                           new Breadcrumb<>(SINGLETON_TREE, empty(), empty()));
   }

   @Test
   public void testToStringContainsUsefulInformation()
   {
      assertTrue(new Tree<>("ABCD1234", empty()).getFocus().toString().contains("ABCD1234"));
      String childFocusString = BINARY_TREE.getFocus().firstChild().map(TreeFocus::toString).orElse("");
      assertTrue(childFocusString.contains("1") && childFocusString.contains("0") && childFocusString.contains("2")); // both root and siblings
   }


   @Test public void testRootOfRootFocusReturnsTheSameInstance()
   {
      TreeFocus<Tree<Integer>> root = BINARY_TREE.getFocus();
      assertTrue(root == root.root());
   }

   @Test public void testRootHasNoParent()
   {
      TreeFocus<Tree<Integer>> root = BINARY_TREE.getFocus();
      assertFalse(root.parent().isPresent());
   }

   @Test public void testSingletonHasNoChildren()
   {
      assertFalse(SINGLETON_TREE.getFocus().firstChild().isPresent());
   }

   @Test public void testFirstChildReturnsTheCorrectChild()
   {
      TreeFocus<Tree<Integer>> root = BINARY_TREE.getFocus();
      Optional<TreeFocus<Tree<Integer>>> firstChild = root.firstChild();
      assertTrue(firstChild.isPresent());
      assertEquals(1, (int)firstChild.get().getFocusedNode().getValue());
   }

   @Test public void testSiblingOfFirstChildIsTheSecondChild()
   {
      TreeFocus<Tree<Integer>> root = BINARY_TREE.getFocus();
      Optional<TreeFocus<Tree<Integer>>> firstChild = root.firstChild();
      Optional<TreeFocus<Tree<Integer>>> secondChild = firstChild.flatMap(TreeFocus::nextSibling);
      assertTrue(secondChild.isPresent());
      assertEquals(2, (int)secondChild.get().getFocusedNode().getValue());
   }

   @Test public void testSiblingOfSecondChildIsTheFirstChild()
   {
      TreeFocus<Tree<Integer>> root = BINARY_TREE.getFocus();
      Optional<TreeFocus<Tree<Integer>>> secondChild = root.firstChild().flatMap(TreeFocus::nextSibling);
      Optional<TreeFocus<Tree<Integer>>> firstChild = secondChild.flatMap(TreeFocus::previousSibling);
      assertTrue(secondChild.isPresent());
      assertEquals(root.firstChild(), firstChild);
   }

   @Test public void testRootFindsTreeRootFromChild()
   {
      Optional<TreeFocus<Tree<Integer>>> firstChild = BINARY_TREE.getFocus().firstChild();
      assertTrue(firstChild.isPresent());
      assertEquals(BINARY_TREE.getFocus(), firstChild.get().root());
   }

   @Test public void testRootHasNoSiblings()
   {
      TreeFocus<Tree<Integer>> root = BINARY_TREE.getFocus();
      assertFalse(root.nextSibling().isPresent());
      assertFalse(root.previousSibling().isPresent());
   }

   @Test public void testFindLocatesChildNode()
   {
      TreeFocus<Tree<Integer>> root = BINARY_TREE.getFocus();
      Optional<TreeFocus<Tree<Integer>>> found1 = root.findChild(node -> node.getValue() == 1);
      assertTrue(found1.isPresent() && found1.get().getFocusedNode().getValue() == 1);

      Optional<TreeFocus<Tree<Integer>>> found2 = root.findChild(node -> node.getValue() == 2);
      assertTrue(found2.isPresent() && found2.get().getFocusedNode().getValue() == 2);
   }

}
