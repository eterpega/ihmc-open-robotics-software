package us.ihmc.robotbuilder.util;

import javafx.scene.control.TreeItem;
import javaslang.collection.List;
import org.junit.Test;
import us.ihmc.robotbuilder.util.TreeFocus.Breadcrumb;
import us.ihmc.robotbuilder.util.TreeFocus.ChildIndexOf;
import us.ihmc.tools.testing.JUnitTools;

import java.util.Iterator;
import java.util.Optional;

import static javaslang.collection.List.empty;
import static org.junit.Assert.*;

/**
 *
 */
public class TreeFocusTest
{
   private static final Tree<Integer> BINARY_TREE = tree(0, tree(1), tree(2));
   private static final Tree<Integer> SINGLETON_TREE = tree(0);
   private static final Tree<Integer> DEEP_TREE = tree(0, tree(1, tree(3), tree(4), tree(5)), tree(2));

   private static Tree<Integer> tree(int value, Tree... children)
   {
      //noinspection unchecked
      return new Tree<>(value, List.of((Tree<Integer>[])children));
   }

   @Test public void testEqualsHashCode()
   {
      //noinspection OptionalGetWithoutIsPresent
      JUnitTools.testHashCodeEqualsMethods(BINARY_TREE.getFocus(), BINARY_TREE.getFocus(), BINARY_TREE.getFocus().firstChild().get());
      JUnitTools.testHashCodeEqualsMethods(new Breadcrumb<>(BINARY_TREE, empty(), empty()),
                                           new Breadcrumb<>(BINARY_TREE, empty(), empty()),
                                           new Breadcrumb<>(SINGLETON_TREE, empty(), empty()));
   }

   @Test public void testToStringContainsUsefulInformation()
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

   @Test public void testFocusOnPathReturnsRootWithEmptyPath()
   {
      Optional<TreeFocus<Tree<Integer>>> rootFocus = SINGLETON_TREE.getFocus().focusOnPath(empty());
      assertTrue(rootFocus.isPresent());
      assertEquals(rootFocus, Optional.of(SINGLETON_TREE.getFocus()));
   }

   @Test public void testFocusOnPathReturnsCorrectNode()
   {
      Optional<TreeFocus<Tree<Integer>>> focus = DEEP_TREE.getFocus().focusOnPath(List.of(0, 1));
      assertTrue(focus.isPresent());
      assertEquals(4, (int)focus.get().getFocusedNode().getValue());
   }

   @Test public void testFocusOnInvalidPathReturnsEmptyFocus()
   {
      Optional<TreeFocus<Tree<Integer>>> focus = DEEP_TREE.getFocus().focusOnPath(List.of(0, 100000));
      assertFalse(focus.isPresent());
   }

   private static final ChildIndexOf<TreeItem<Object>> treeItemIndexOf = (parent, child) -> parent.getChildren().indexOf(child);

   @Test public void testPathToRootReturnsEmptyPath()
   {
      List<Integer> path = TreeFocus.pathTo(new TreeItem<>(), treeItemIndexOf, TreeItem::getParent);
      assertTrue(path.isEmpty());
   }

   @Test public void testPathToFirstChildReturnsOneItem()
   {
      TreeItem<Object> root = new TreeItem<>();
      root.getChildren().add(new TreeItem<>());
      List<Integer> path = TreeFocus.pathTo(root.getChildren().get(0), treeItemIndexOf, TreeItem::getParent);
      assertEquals(List.of(0), path);
   }

   @Test public void testPathToDeepChildIsCorrectlyOrdered()
   {
      TreeItem<Object> deepTree = DEEP_TREE.map((node, children) ->
         {
            TreeItem<Object> result = new TreeItem<>(node.getValue());
            result.getChildren().addAll(children);
            return result;
         }
      );

      TreeItem<Object> childToFind = deepTree.getChildren().get(0).getChildren().get(1);
      List<Integer> path = TreeFocus.pathTo(childToFind, treeItemIndexOf, TreeItem::getParent);

      assertEquals(List.of(0, 1), path);
   }

   @Test public void testPathToInvalidChildReturnsEmptyPath()
   {
      TreeItem<Object> root = new TreeItem<>(), child = new TreeItem<>();
      child.getChildren().add(new TreeItem<>());
      root.getChildren().add(child);
      List<Integer> path = TreeFocus.pathTo(child.getChildren().get(0), (x, y) -> -1, TreeItem::getParent);
      assertEquals(List.empty(), path);
   }

   @Test public void testFirstChildReturnsTheCorrectChild()
   {
      TreeFocus<Tree<Integer>> root = BINARY_TREE.getFocus();
      Optional<TreeFocus<Tree<Integer>>> firstChild = root.firstChild();
      assertTrue(firstChild.isPresent());
      assertEquals(1, (int) firstChild.get().getFocusedNode().getValue());
   }

   @Test public void testGetChildReturnsEmptyForInvalidChildIndex()
   {
      TreeFocus<Tree<Integer>> root = BINARY_TREE.getFocus();
      assertFalse(root.getChild(-1).isPresent());
      assertFalse(root.getChild(2).isPresent());
   }

   @Test public void testGetChildReturnsChildAtTheCorrectLocation()
   {
      TreeFocus<Tree<Integer>> root = BINARY_TREE.getFocus();
      assertEquals(root.getChild(0), root.firstChild());
      assertEquals(root.getChild(1), root.firstChild().flatMap(TreeFocus::nextSibling));
   }

   @Test public void testSiblingOfFirstChildIsTheSecondChild()
   {
      TreeFocus<Tree<Integer>> root = BINARY_TREE.getFocus();
      Optional<TreeFocus<Tree<Integer>>> firstChild = root.firstChild();
      Optional<TreeFocus<Tree<Integer>>> secondChild = firstChild.flatMap(TreeFocus::nextSibling);
      assertTrue(firstChild.isPresent());
      assertTrue(secondChild.isPresent());
      assertEquals(2, (int) secondChild.get().getFocusedNode().getValue());
      assertTrue(firstChild.get().root().getFocusedNode() == BINARY_TREE);
      assertTrue(secondChild.get().root().getFocusedNode() == BINARY_TREE);
   }

   @Test public void testSiblingOfSecondChildIsTheFirstChild()
   {
      TreeFocus<Tree<Integer>> root = BINARY_TREE.getFocus();
      Optional<TreeFocus<Tree<Integer>>> secondChild = root.firstChild().flatMap(TreeFocus::nextSibling);
      Optional<TreeFocus<Tree<Integer>>> firstChild = secondChild.flatMap(TreeFocus::previousSibling);
      assertTrue(secondChild.isPresent());
      assertEquals(root.firstChild(), firstChild);
   }

   @Test public void testRootMethodFindsTreeRootFromChild()
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

   @Test public void testReplaceCreatesANewTreeWithTheReplacedNode()
   {
      TreeFocus<Tree<Integer>> root = DEEP_TREE.getFocus();
      root.firstChild().map(child ->
                            {
                               TreeFocus<Tree<Integer>> replaced = child.replace(new Tree<>(42));
                               TreeFocus<Tree<Integer>> replacedRoot = replaced.root();
                               assertNotEquals(replacedRoot.getFocusedNode(), root);
                               assertEquals(42, (int)replacedRoot.getFocusedNode().getChild(0).getValue());
                               assertFalse(replacedRoot.getFocusedNode().getChild(0).firstChild().isPresent());
                               return child;
                            })
      .orElseGet(() -> {
         fail("Root has no children");
         return null;
      });
   }

   @Test public void testAddSiblingDoesNothingOnRootNode()
   {
      TreeFocus<Tree<Integer>> root = BINARY_TREE.getFocus();
      assertFalse(root.addLeftSibling(new Tree<>(1)).isPresent());
      assertFalse(root.addRightSibling(new Tree<>(1)).isPresent());
   }

   @Test public void testAddLeftSiblingAtTheBeginning()
   {
      BINARY_TREE.getFocus().firstChild()
            .flatMap(firstChild -> firstChild.addLeftSibling(new Tree<>(3)))
            .map(addedSibling -> {
               assertEquals(3, (int)addedSibling.getFocusedNode().getValue());
               assertNotEquals(addedSibling.root(), BINARY_TREE.getFocus());

               assertEquals(BINARY_TREE.firstChild(), addedSibling.nextSibling().map(TreeFocus::getFocusedNode));

               assertFalse(addedSibling.previousSibling().isPresent());
               return addedSibling;
            })
      .orElseGet(() -> {
         assertTrue("Root has no children", false);
         return null;
      });
   }

   @Test public void testAddLeftSiblingAtTheEnd()
   {
      BINARY_TREE.getFocus().firstChild()
                 .flatMap(TreeFocus::nextSibling)
                 .flatMap(lastChild -> lastChild.addLeftSibling(new Tree<>(3)))
                 .map(addedSibling -> {
                    assertEquals(3, (int)addedSibling.getFocusedNode().getValue());
                    assertNotEquals(addedSibling.root(), BINARY_TREE.getFocus());

                    assertEquals(Optional.of(1), addedSibling.previousSibling().map(TreeFocus::getFocusedNode).map(Tree::getValue));
                    assertEquals(Optional.of(2), addedSibling.nextSibling().map(TreeFocus::getFocusedNode).map(Tree::getValue));
                    return addedSibling;
                 })
                 .orElseGet(() -> {
                    assertTrue("Root has no children", false);
                    return null;
                 });
   }

   @Test public void testAddRightSiblingAtTheBeginning()
   {
      BINARY_TREE.getFocus().firstChild()
                 .flatMap(firstChild -> firstChild.addRightSibling(new Tree<>(3)))
                 .map(addedSibling -> {
                    assertEquals(3, (int)addedSibling.getFocusedNode().getValue());
                    assertNotEquals(addedSibling.root(), BINARY_TREE.getFocus());

                    assertEquals(Optional.of(1), addedSibling.previousSibling().map(TreeFocus::getFocusedNode).map(Tree::getValue));
                    assertEquals(Optional.of(2), addedSibling.nextSibling().map(TreeFocus::getFocusedNode).map(Tree::getValue));
                    return addedSibling;
                 })
                 .orElseGet(() -> {
                    assertTrue("Root has no children", false);
                    return null;
                 });
   }

   @Test public void testAddRightSiblingAtTheEnd()
   {
      BINARY_TREE.getFocus().firstChild()
                 .flatMap(firstChild -> firstChild.addLeftSibling(new Tree<>(3)))
                 .map(addedSibling -> {
                    assertEquals(3, (int)addedSibling.getFocusedNode().getValue());
                    assertNotEquals(addedSibling.root(), BINARY_TREE.getFocus());

                    assertEquals(BINARY_TREE.firstChild(), addedSibling.nextSibling().map(TreeFocus::getFocusedNode));

                    assertFalse(addedSibling.previousSibling().isPresent());
                    return addedSibling;
                 })
                 .orElseGet(() -> {
                    assertTrue("Root has no children", false);
                    return null;
                 });
   }

   @Test public void testAppendChild()
   {
      Tree<Integer> newChild = new Tree<>(3);
      TreeFocus<Tree<Integer>> newChildFocus = BINARY_TREE.getFocus().appendChild(newChild);

      Tree<Integer> newRoot = newChildFocus.root().getFocusedNode();
      assertEquals(3, newRoot.countChildren());

      assertEquals(newRoot.findValue(x -> x == 3)
                          .flatMap(TreeFocus::previousSibling)
                          .map(TreeFocus::getFocusedNode)
                          .map(Tree::getValue),
                   Optional.of(2));
   }

   @Test public void testPrependChild()
   {
      Tree<Integer> newChild = new Tree<>(3);
      TreeFocus<Tree<Integer>> newChildFocus = BINARY_TREE.getFocus().prependChild(newChild);

      Tree<Integer> newRoot = newChildFocus.root().getFocusedNode();
      assertEquals(3, newRoot.childStream().count());

      assertEquals(newRoot.findValue(x -> x == 3)
                          .flatMap(TreeFocus::nextSibling)
                          .map(TreeFocus::getFocusedNode)
                          .map(Tree::getValue),
                   Optional.of(1));
   }

   @Test public void testRemoveRootReturnsEmptyFocus()
   {
      assertFalse(BINARY_TREE.getFocus().remove().isPresent());
   }

   @Test public void testRemoveChildReturnsRootWithoutTheChild()
   {
      BINARY_TREE.getFocus()
                 .firstChild()
                 .flatMap(TreeFocus::remove)
      .map(removedParent -> {
         assertEquals(1, removedParent.getFocusedNode().childStream().count());
         assertEquals(2, (int)removedParent.getFocusedNode().getChild(0).getValue());
         return removedParent;
      }).orElseGet(() -> {
         assertTrue("Removed node should have a parent", false);
         return null;
      });
   }

   @Test public void testChildIteratorOnLeafNodeHasNoNextItem()
   {
      assertFalse(SINGLETON_TREE.getFocus().getChildren().iterator().hasNext());
   }

   @Test public void testChildIteratorVisitsAllChildren()
   {
      Iterator<Tree<Integer>> baseIter = BINARY_TREE.getChildren().iterator();
      Iterator<TreeFocus<Tree<Integer>>> focusIter = BINARY_TREE.getFocus().getChildren().iterator();
      while (baseIter.hasNext())
      {
         assertTrue(focusIter.hasNext());
         assertEquals(baseIter.next(), focusIter.next().getFocusedNode());
      }
   }

   @Test public void testIndexOutOfBoundsIsThrownForAnInvalidIteratorAccess()
   {
      Iterator<TreeFocus<Tree<Integer>>> focusIter = BINARY_TREE.getFocus().getChildren().iterator();
      focusIter.next();
      focusIter.next();
      try {
         focusIter.next();
         fail("Index out of bounds should have been thrown");
      } catch (IndexOutOfBoundsException ex) {
         // OK
      }

      focusIter = SINGLETON_TREE.getFocus().getChildren().iterator();
      try {
         focusIter.next();
         fail("Index out of bounds should have been thrown for a singleton list");
      } catch (IndexOutOfBoundsException ex) {
         // OK
      }
   }
}
