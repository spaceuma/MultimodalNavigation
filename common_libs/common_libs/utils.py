from collections import deque

class Stack:
  def __init__(self):
    self._elements = deque()

  def __len__(self):
    return len(self._elements)

  def __iter__(self):
    while len(self) > 0:
      yield self.dequeue()

  def enqueue(self, element):
    self._elements.append(element)

  def dequeue(self):
    return self._elements.pop()
  
  def clear(self, num_to_keep = 0):
    if num_to_keep >= 0:
      self._elements = deque(list(self._elements)[-num_to_keep:])