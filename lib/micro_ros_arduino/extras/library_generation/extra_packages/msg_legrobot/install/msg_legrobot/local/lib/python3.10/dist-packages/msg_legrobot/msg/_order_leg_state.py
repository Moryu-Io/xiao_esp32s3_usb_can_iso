# generated from rosidl_generator_py/resource/_idl.py.em
# with input from msg_legrobot:msg/OrderLegState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'force_lf'
# Member 'pos_lf'
# Member 'force_rf'
# Member 'pos_rf'
# Member 'force_lr'
# Member 'pos_lr'
# Member 'force_rr'
# Member 'pos_rr'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_OrderLegState(type):
    """Metaclass of message 'OrderLegState'."""

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
            module = import_type_support('msg_legrobot')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'msg_legrobot.msg.OrderLegState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__order_leg_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__order_leg_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__order_leg_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__order_leg_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__order_leg_state

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class OrderLegState(metaclass=Metaclass_OrderLegState):
    """Message class 'OrderLegState'."""

    __slots__ = [
        '_mode_lf',
        '_force_lf',
        '_pos_lf',
        '_mode_rf',
        '_force_rf',
        '_pos_rf',
        '_mode_lr',
        '_force_lr',
        '_pos_lr',
        '_mode_rr',
        '_force_rr',
        '_pos_rr',
    ]

    _fields_and_field_types = {
        'mode_lf': 'uint32',
        'force_lf': 'float[3]',
        'pos_lf': 'float[3]',
        'mode_rf': 'uint32',
        'force_rf': 'float[3]',
        'pos_rf': 'float[3]',
        'mode_lr': 'uint32',
        'force_lr': 'float[3]',
        'pos_lr': 'float[3]',
        'mode_rr': 'uint32',
        'force_rr': 'float[3]',
        'pos_rr': 'float[3]',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 3),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 3),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 3),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 3),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 3),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 3),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 3),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 3),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.mode_lf = kwargs.get('mode_lf', int())
        if 'force_lf' not in kwargs:
            self.force_lf = numpy.zeros(3, dtype=numpy.float32)
        else:
            self.force_lf = numpy.array(kwargs.get('force_lf'), dtype=numpy.float32)
            assert self.force_lf.shape == (3, )
        if 'pos_lf' not in kwargs:
            self.pos_lf = numpy.zeros(3, dtype=numpy.float32)
        else:
            self.pos_lf = numpy.array(kwargs.get('pos_lf'), dtype=numpy.float32)
            assert self.pos_lf.shape == (3, )
        self.mode_rf = kwargs.get('mode_rf', int())
        if 'force_rf' not in kwargs:
            self.force_rf = numpy.zeros(3, dtype=numpy.float32)
        else:
            self.force_rf = numpy.array(kwargs.get('force_rf'), dtype=numpy.float32)
            assert self.force_rf.shape == (3, )
        if 'pos_rf' not in kwargs:
            self.pos_rf = numpy.zeros(3, dtype=numpy.float32)
        else:
            self.pos_rf = numpy.array(kwargs.get('pos_rf'), dtype=numpy.float32)
            assert self.pos_rf.shape == (3, )
        self.mode_lr = kwargs.get('mode_lr', int())
        if 'force_lr' not in kwargs:
            self.force_lr = numpy.zeros(3, dtype=numpy.float32)
        else:
            self.force_lr = numpy.array(kwargs.get('force_lr'), dtype=numpy.float32)
            assert self.force_lr.shape == (3, )
        if 'pos_lr' not in kwargs:
            self.pos_lr = numpy.zeros(3, dtype=numpy.float32)
        else:
            self.pos_lr = numpy.array(kwargs.get('pos_lr'), dtype=numpy.float32)
            assert self.pos_lr.shape == (3, )
        self.mode_rr = kwargs.get('mode_rr', int())
        if 'force_rr' not in kwargs:
            self.force_rr = numpy.zeros(3, dtype=numpy.float32)
        else:
            self.force_rr = numpy.array(kwargs.get('force_rr'), dtype=numpy.float32)
            assert self.force_rr.shape == (3, )
        if 'pos_rr' not in kwargs:
            self.pos_rr = numpy.zeros(3, dtype=numpy.float32)
        else:
            self.pos_rr = numpy.array(kwargs.get('pos_rr'), dtype=numpy.float32)
            assert self.pos_rr.shape == (3, )

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
        if self.mode_lf != other.mode_lf:
            return False
        if all(self.force_lf != other.force_lf):
            return False
        if all(self.pos_lf != other.pos_lf):
            return False
        if self.mode_rf != other.mode_rf:
            return False
        if all(self.force_rf != other.force_rf):
            return False
        if all(self.pos_rf != other.pos_rf):
            return False
        if self.mode_lr != other.mode_lr:
            return False
        if all(self.force_lr != other.force_lr):
            return False
        if all(self.pos_lr != other.pos_lr):
            return False
        if self.mode_rr != other.mode_rr:
            return False
        if all(self.force_rr != other.force_rr):
            return False
        if all(self.pos_rr != other.pos_rr):
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def mode_lf(self):
        """Message field 'mode_lf'."""
        return self._mode_lf

    @mode_lf.setter
    def mode_lf(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'mode_lf' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'mode_lf' field must be an unsigned integer in [0, 4294967295]"
        self._mode_lf = value

    @builtins.property
    def force_lf(self):
        """Message field 'force_lf'."""
        return self._force_lf

    @force_lf.setter
    def force_lf(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'force_lf' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 3, \
                "The 'force_lf' numpy.ndarray() must have a size of 3"
            self._force_lf = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 3 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'force_lf' field must be a set or sequence with length 3 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._force_lf = numpy.array(value, dtype=numpy.float32)

    @builtins.property
    def pos_lf(self):
        """Message field 'pos_lf'."""
        return self._pos_lf

    @pos_lf.setter
    def pos_lf(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'pos_lf' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 3, \
                "The 'pos_lf' numpy.ndarray() must have a size of 3"
            self._pos_lf = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 3 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'pos_lf' field must be a set or sequence with length 3 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._pos_lf = numpy.array(value, dtype=numpy.float32)

    @builtins.property
    def mode_rf(self):
        """Message field 'mode_rf'."""
        return self._mode_rf

    @mode_rf.setter
    def mode_rf(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'mode_rf' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'mode_rf' field must be an unsigned integer in [0, 4294967295]"
        self._mode_rf = value

    @builtins.property
    def force_rf(self):
        """Message field 'force_rf'."""
        return self._force_rf

    @force_rf.setter
    def force_rf(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'force_rf' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 3, \
                "The 'force_rf' numpy.ndarray() must have a size of 3"
            self._force_rf = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 3 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'force_rf' field must be a set or sequence with length 3 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._force_rf = numpy.array(value, dtype=numpy.float32)

    @builtins.property
    def pos_rf(self):
        """Message field 'pos_rf'."""
        return self._pos_rf

    @pos_rf.setter
    def pos_rf(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'pos_rf' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 3, \
                "The 'pos_rf' numpy.ndarray() must have a size of 3"
            self._pos_rf = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 3 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'pos_rf' field must be a set or sequence with length 3 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._pos_rf = numpy.array(value, dtype=numpy.float32)

    @builtins.property
    def mode_lr(self):
        """Message field 'mode_lr'."""
        return self._mode_lr

    @mode_lr.setter
    def mode_lr(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'mode_lr' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'mode_lr' field must be an unsigned integer in [0, 4294967295]"
        self._mode_lr = value

    @builtins.property
    def force_lr(self):
        """Message field 'force_lr'."""
        return self._force_lr

    @force_lr.setter
    def force_lr(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'force_lr' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 3, \
                "The 'force_lr' numpy.ndarray() must have a size of 3"
            self._force_lr = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 3 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'force_lr' field must be a set or sequence with length 3 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._force_lr = numpy.array(value, dtype=numpy.float32)

    @builtins.property
    def pos_lr(self):
        """Message field 'pos_lr'."""
        return self._pos_lr

    @pos_lr.setter
    def pos_lr(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'pos_lr' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 3, \
                "The 'pos_lr' numpy.ndarray() must have a size of 3"
            self._pos_lr = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 3 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'pos_lr' field must be a set or sequence with length 3 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._pos_lr = numpy.array(value, dtype=numpy.float32)

    @builtins.property
    def mode_rr(self):
        """Message field 'mode_rr'."""
        return self._mode_rr

    @mode_rr.setter
    def mode_rr(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'mode_rr' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'mode_rr' field must be an unsigned integer in [0, 4294967295]"
        self._mode_rr = value

    @builtins.property
    def force_rr(self):
        """Message field 'force_rr'."""
        return self._force_rr

    @force_rr.setter
    def force_rr(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'force_rr' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 3, \
                "The 'force_rr' numpy.ndarray() must have a size of 3"
            self._force_rr = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 3 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'force_rr' field must be a set or sequence with length 3 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._force_rr = numpy.array(value, dtype=numpy.float32)

    @builtins.property
    def pos_rr(self):
        """Message field 'pos_rr'."""
        return self._pos_rr

    @pos_rr.setter
    def pos_rr(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'pos_rr' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 3, \
                "The 'pos_rr' numpy.ndarray() must have a size of 3"
            self._pos_rr = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 3 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'pos_rr' field must be a set or sequence with length 3 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._pos_rr = numpy.array(value, dtype=numpy.float32)
