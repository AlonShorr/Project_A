"""implements a Union-Find data structure

Implementation based on https://github.com/mrapacz/disjoint-set, but adds union-by-size
and provides different iteration functions.
"""

from __future__ import annotations

from collections import Counter, defaultdict
from collections.abc import Iterator
from typing import Any, Literal, overload, TYPE_CHECKING


__all__ = ['InvalidInitialMappingError', 'UnionFind']

# pylint 3.0.3 currently has bugs regarding the [T] syntax to create type variables.
# pylint 3.0.3 currently has bugs regarding the use of @overload -> it thinks the function returns None.
# Disable the problematic errors globally in this file:
# pylint: disable=undefined-variable,not-an-iterable


class IdentityDict[T](dict[T, T]):
    """A defaultdict implementation which places the requested key as its value in case it's missing."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def __missing__(self, key: T) -> T:
        self[key] = key
        return key


class OneBasedCounter(Counter):
    """
    A collections.Counter that starts new items from 1 instead of 0.

    >>> c = OneBasedCounter('abcdeabcdabcaba')  # count elements from a string

    >>> c.most_common(3)                # three most common elements
    [('a', 5), ('b', 4), ('c', 3)]

    >>> c['q']
    1
    """

    def __missing__(self, key):
        return 1

    if TYPE_CHECKING:
        @classmethod
        def fromkeys(cls, iterable, v=None):
            # This is defined in Counter by pylint decided it's an unimplemented *abstract* function
            raise NotImplementedError('Counter.fromkeys() is undefined.  Use OneBasedCounter(iterable) instead.')


class InvalidInitialMappingError(RuntimeError):
    """Runtime error raised when invalid initial mapping causes the find() methods to change during iteration."""

    def __init__(
            self,
            msg=(
                "The mapping passed during the UnionFind initialization must have been wrong. "
                "Check that all keys are mapping to other keys and not some external values."
            ),
            /,
            *args,
            **kwargs,
    ):
        super().__init__(msg, *args, **kwargs)


class UnionFind[T]:
    """
    A union-find (disjoint-set) data structure.
    """

    def __init__(self, *args, **kwargs) -> None:
        self._data: IdentityDict[T] = IdentityDict(*args, **kwargs)
        self._set_sizes = OneBasedCounter(self._data.values())

    def __contains__(self, item: T) -> bool:
        return item in self._data

    def __bool__(self) -> bool:
        return bool(self._data)

    def __getitem__(self, element: T) -> T:
        return self.find(element)

    def _sets_set(self) -> frozenset[frozenset[T]]:
        """
        Return a frozenset of frozensets of the values in each set in the UnionFind.
        """
        return frozenset({frozenset(x) for x in self.iter_sets()})

    def __eq__(self, other: Any) -> bool:
        """
        Return True if both DistjoinSet structures are equivalent.

        This may mean that their canonical elements are different, but the sets they form are the same.
        >>> UnionFind({1: 1, 2: 1}) == UnionFind({1: 2, 2: 2})
        True
        """
        if not isinstance(other, UnionFind):
            return NotImplemented

        return self._sets_set() == other._sets_set()

    def __repr__(self) -> str:
        """
        Print self in a reproducible way.

        >>> UnionFind({1: 2, 2: 2})
        UnionFind({1: 2, 2: 2})
        """
        return f"{self.__class__.__name__}({dict(self)})"

    def __str__(self) -> str:
        return f"{self.__class__.__name__}({', '.join(str(uf) for uf in self.iter_sets())})"

    def __iter__(self) -> Iterator[tuple[T, T]]:
        """Iterate over items and their canonical elements."""
        try:
            for key in self._data.keys():
                yield key, self.find(key)
        except RuntimeError as e:
            raise InvalidInitialMappingError from e

    @overload
    def iter_sets(self) -> Iterator[set[T]]: ...
    @overload
    def iter_sets(self, with_canonical_elements: Literal[False]) -> Iterator[set[T]]: ...
    @overload
    def iter_sets(self, with_canonical_elements: Literal[True]) -> Iterator[tuple[T, set[T]]]: ...

    def iter_sets(self, with_canonical_elements: bool = False) -> Iterator[set[T]] | Iterator[tuple[T, set[T]]]:
        """
        Yield sets of connected components.

        If with_canonical_elements is set to True, method will yield tuples of (<canonical_element>, <set of elements>)
        >>> uf = UnionFind()
        >>> uf.union(1, 2)
        >>> list(uf.iter_sets())
        [{1, 2}]
        >>> list(uf.iter_sets(with_canonical_elements=True))
        [(1, {1, 2})]
        """
        element_classes: dict[T, set[T]] = defaultdict(set)
        for element in self._data:
            element_classes[self.find(element)].add(element)

        if with_canonical_elements:
            yield from element_classes.items()
        else:
            yield from element_classes.values()

    def find(self, x: T) -> T:
        """
        Return the canonical element of a given item.

        In case the element was not present in the data structure, the canonical element is the item itself.
        >>> uf = UnionFind()
        >>> uf.find(2)
        2
        >>> uf.union(1, 2)
        >>> uf.find(2)
        1
        """
        while x != self._data[x]:
            self._data[x] = self._data[self._data[x]]
            x = self._data[x]
        return x

    def union(self, x: T, y: T) -> None:
        """
        Attach the roots of x and y trees together if they are not the same already.

        :param x: first element
        :param y: second element
        """
        parent_x, parent_y = self.find(x), self.find(y)
        if parent_x == parent_y:
            return
        if self._set_sizes[parent_x] < self._set_sizes[parent_y]:
            parent_x, parent_y = parent_y, parent_x
        self._data[parent_y] = parent_x
        self._set_sizes[parent_x] += self._set_sizes[parent_y]

    def connected(self, x: T, y: T) -> bool:
        """
        Return True if x and y belong to the same set (i.e. they have the canonical element).

        >>> uf = UnionFind()
        >>> uf.connected(1, 2)
        False
        >>> uf.union(1, 2)
        >>> uf.connected(1, 2)
        True
        """
        return self.find(x) == self.find(y)
