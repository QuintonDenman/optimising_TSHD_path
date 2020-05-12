class Node(object):
    def __init__(self, name, value):
        self.name = name
        self.value = value
        self.children = []
    def add_child(self, obj):
        self.children.append(obj)