from io import StringIO

_XML_TREE_BEGIN_TEMPLATE = "<{kind}{params}>\n"
_XML_LEAF_BEGIN_TEMPLATE = "<{kind}{params}/>\n"
_XML_TREE_END_TEMPLATE = "</{kind}>\n"
_XML_VERSION_TEMPLATE = "<?xml version={version}?>\n"

_parent_stack = [None]


def push_parent(root):
    _parent_stack.append(root)


def pop_parent():
    return _parent_stack.pop()


def peek_parent():
    return _parent_stack[-1]


class XMLNode:
    def __init__(self, kind, fields=None):
        if not fields:
            fields = {}

        self.kind = kind
        self.fields = fields

        self._previous_root = None

    def __enter__(self):
        self._previous_root = pop_parent()
        push_parent(self)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        pop_parent()
        push_parent(self._previous_root)


class TreeNode(XMLNode):
    def __init__(self, kind, fields=None):
        super().__init__(kind, fields)
        self.children = []


class LeafNode(XMLNode):
    pass


def node(kind, **fields):
    node = TreeNode(kind, fields)

    parent = peek_parent()
    if parent:
        parent.children.append(node)

    return node


def leaf(kind, **fields):
    node = LeafNode(kind, fields)
    peek_parent().children.append(node)


def dict_to_args(dict_):
    return ", ".join("{}={!r}".format(n, v) for n, v in dict_.items())


def document_to_xml(document, version=None, indent='   ', io=None):
    if io is None:
        io = StringIO()

    if version is not None:
        io.write(_XML_VERSION_TEMPLATE.format(version=version))

    node_to_xml(document, io, indent)
    return io.getvalue()

def node_to_xml(node, io, indent, depth=0):
    if hasattr(node, 'children'):
        begin_template = _XML_TREE_BEGIN_TEMPLATE
    else:
        begin_template = _XML_LEAF_BEGIN_TEMPLATE

    margin = indent * depth

    params_str = " " + dict_to_args(node.fields) if node.fields else ""
    io.write("{}{}".format(margin, begin_template.format(kind=node.kind, params=params_str)))

    if hasattr(node, "children"):
        for child in node.children:
            node_to_xml(child, io, indent, depth + 1)

        io.write("{}{}".format(margin, _XML_TREE_END_TEMPLATE.format(kind=node.kind)))
