# generated from rosidl_generator_py/resource/_idl.py.em
# with input from two_wheel_control_msgs:msg/Command.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Command(type):
    """Metaclass of message 'Command'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('two_wheel_control_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'two_wheel_control_msgs.msg.Command')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__command
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__command
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__command
            cls._TYPE_SUPPORT = module.type_support_msg__msg__command
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__command

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Command(metaclass=Metaclass_Command):
    """Message class 'Command'."""

    __slots__ = [
        '_p',
        '_thetay',
        '_thetaz',
        '_p_dot',
        '_thetay_dot',
        '_thetaz_dot',
        '_p_ddot',
        '_thetay_ddot',
        '_thetaz_ddot',
    ]

    _fields_and_field_types = {
        'p': 'double',
        'thetay': 'double',
        'thetaz': 'double',
        'p_dot': 'double',
        'thetay_dot': 'double',
        'thetaz_dot': 'double',
        'p_ddot': 'double',
        'thetay_ddot': 'double',
        'thetaz_ddot': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.p = kwargs.get('p', float())
        self.thetay = kwargs.get('thetay', float())
        self.thetaz = kwargs.get('thetaz', float())
        self.p_dot = kwargs.get('p_dot', float())
        self.thetay_dot = kwargs.get('thetay_dot', float())
        self.thetaz_dot = kwargs.get('thetaz_dot', float())
        self.p_ddot = kwargs.get('p_ddot', float())
        self.thetay_ddot = kwargs.get('thetay_ddot', float())
        self.thetaz_ddot = kwargs.get('thetaz_ddot', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.p != other.p:
            return False
        if self.thetay != other.thetay:
            return False
        if self.thetaz != other.thetaz:
            return False
        if self.p_dot != other.p_dot:
            return False
        if self.thetay_dot != other.thetay_dot:
            return False
        if self.thetaz_dot != other.thetaz_dot:
            return False
        if self.p_ddot != other.p_ddot:
            return False
        if self.thetay_ddot != other.thetay_ddot:
            return False
        if self.thetaz_ddot != other.thetaz_ddot:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def p(self):
        """Message field 'p'."""
        return self._p

    @p.setter
    def p(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'p' field must be of type 'float'"
        self._p = value

    @property
    def thetay(self):
        """Message field 'thetay'."""
        return self._thetay

    @thetay.setter
    def thetay(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'thetay' field must be of type 'float'"
        self._thetay = value

    @property
    def thetaz(self):
        """Message field 'thetaz'."""
        return self._thetaz

    @thetaz.setter
    def thetaz(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'thetaz' field must be of type 'float'"
        self._thetaz = value

    @property
    def p_dot(self):
        """Message field 'p_dot'."""
        return self._p_dot

    @p_dot.setter
    def p_dot(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'p_dot' field must be of type 'float'"
        self._p_dot = value

    @property
    def thetay_dot(self):
        """Message field 'thetay_dot'."""
        return self._thetay_dot

    @thetay_dot.setter
    def thetay_dot(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'thetay_dot' field must be of type 'float'"
        self._thetay_dot = value

    @property
    def thetaz_dot(self):
        """Message field 'thetaz_dot'."""
        return self._thetaz_dot

    @thetaz_dot.setter
    def thetaz_dot(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'thetaz_dot' field must be of type 'float'"
        self._thetaz_dot = value

    @property
    def p_ddot(self):
        """Message field 'p_ddot'."""
        return self._p_ddot

    @p_ddot.setter
    def p_ddot(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'p_ddot' field must be of type 'float'"
        self._p_ddot = value

    @property
    def thetay_ddot(self):
        """Message field 'thetay_ddot'."""
        return self._thetay_ddot

    @thetay_ddot.setter
    def thetay_ddot(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'thetay_ddot' field must be of type 'float'"
        self._thetay_ddot = value

    @property
    def thetaz_ddot(self):
        """Message field 'thetaz_ddot'."""
        return self._thetaz_ddot

    @thetaz_ddot.setter
    def thetaz_ddot(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'thetaz_ddot' field must be of type 'float'"
        self._thetaz_ddot = value
