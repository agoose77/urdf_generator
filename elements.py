# This is a script to generate URDF models for ROS simulation in rviz
# Maintainer: Julian Gaal github.com/juliangaal
from abc import ABC, abstractmethod

from tree import node, leaf, document_to_xml


class Field(ABC):
    @abstractmethod
    def to_xml(self):
        raise NotImplementedError


class File:
    """Create XML file for ROS urdf model.

    Args:
        'filename': name of file to be saved to. Should end with .urdf.xacro.
        'objects': individual elements, e.g. link, xacro property, joint

    Raises:
           No Error yet
    """

    def __init__(self, *objects, filename='example.urdf.xacro'):
        self.objects = objects
        self.filename = filename

    def save(self):
        footprint_set = False

        with node('robot') as ctx:
            if any(isinstance(f, Link) for f in self.objects):
                leaf("link", name="base_footprint")
                
            for field in self.objects:
                field.to_xml()

            with open(self.filename, "w") as f:
                f.write(document_to_xml(ctx, version=1.0))
            print("\t-- Success -- \nSaved to %s" % self.filename)


class Expression(ABC):
    def _create_operation(self, other, op):
        if not isinstance(other, self.__class__):
            other = Constant(other)
        return Operation(self, other, op)

    def __mul__(self, other):
        return self._create_operation(other, '*')

    def __add__(self, other):
        return self._create_operation(other, '+')

    def __sub__(self, other):
        return self._create_operation(other, '-')

    def __truediv__(self, other):
        return self._create_operation(other, '/')

    def __mod__(self, other):
        return self._create_operation(other, '%')

    def __pow__(self, other):
        return self._create_operation(other, '**')

    @abstractmethod
    def to_string(self):
        raise NotImplementedError


class Constant(Expression):
    def __init__(self, value):
        self.value = value

    def to_string(self):
        return repr(self.value)


class Operation(Expression):
    def __init__(self, left, right, op):
        self.left = left
        self.right = right
        self.op = op

    def to_string(self):
        return "({} {} {})".format(self.left.to_string(), self.op, self.right.to_string())


def parse_expression(value):
    if hasattr(value, 'to_string'):
        value = "${{{}}}".format(value.to_string())
    return value


class XacroProperty(Field, Expression):
    def __init__(self, name, value):
        self.name = name
        self.value = value

    def to_string(self):
        return self.name

    def to_xml(self):
        name = parse_expression(self.name)
        value = parse_expression(self.value)

        leaf("xacro:property", name=name, value=value)


class XacroInclude(Field):
    def __init__(self, filename):
        self.filename = filename

    def to_xml(self):
        leaf("xacro:include", filename=parse_expression(self.filename))


class Link(Field):
    """Create 'link' urdf element.

    Args:
        rgba: red - green - blue - alpha.
        size: size of object.

    Raises:
           No Error yet
    """

    def __init__(self, name, type_, rgba, size, color, rpy='0 0 0'):
        self.name = name
        self.rpy = rpy
        self.type = type_
        self.rgba = rgba
        self.size = size
        self.color = color

    # Formatting options, see http://tinyurl.com/y9vohsxz

    # Manual xml string, as backup: '#'\n\n<link name="test1">\n\t<visual>\n\t\t<geometry>\n\t\t\t<box size="0.6 0.3 0.2"/>\n\t\t</geometry>
    # \n\t\t<material name="green">\n\t\t\t<color rgba="0.1 0.3 0.2"/>\n\t\t</material>\n\t</visual>\n</link>''

    # ISSUE: String after closing brackets of material need to be indented one space more. wtf
    def to_xml(self):
        name = parse_expression(self.name)
        size = parse_expression(self.size)
        rpy = parse_expression(self.rpy)
        type_ = parse_expression(self.type)
        color = parse_expression(self.color)
        rgba = parse_expression(self.rgba)

        with node("link", name=name):
            with node("visual"):
                with node("geometry"):
                    leaf(type_, size=size, rpy=rpy)
                with node("material", name=color):
                    leaf("color", rgba=rgba)


class Joint(Field):
    def __init__(self, name, type_, parent, child, xyz, rpy='0 0 0'):
        self.name = name
        self.type = type_
        self.parent = parent
        self.child = child
        self.xyz = xyz
        self.rpy = rpy

    def say(self, text):
        print(text)

    def to_xml(self):
        name = parse_expression(self.name)
        parent = parse_expression(self.parent)
        child = parse_expression(self.child)
        xyz = parse_expression(self.xyz)
        rpy = parse_expression(self.rpy)

        with node("joint", name=name):
            leaf('parent', link=parent)
            leaf('child', link=child)
            leaf('origin', xyz=xyz, rpy=rpy)
