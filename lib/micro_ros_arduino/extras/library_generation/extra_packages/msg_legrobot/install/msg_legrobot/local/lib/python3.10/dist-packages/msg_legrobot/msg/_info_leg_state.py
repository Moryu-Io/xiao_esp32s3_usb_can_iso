# generated from rosidl_generator_py/resource/_idl.py.em
# with input from msg_legrobot:msg/InfoLegState.idl
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
# Member 'pos_body'
# Member 'vel_body'
# Member 'acc_imu'
# Member 'gyro_imu'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_InfoLegState(type):
    """Metaclass of message 'InfoLegState'."""

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
                'msg_legrobot.msg.InfoLegState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__info_leg_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__info_leg_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__info_leg_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__info_leg_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__info_leg_state

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class InfoLegState(metaclass=Metaclass_InfoLegState):
    """Message class 'InfoLegState'."""

    __slots__ = [
        '_force_lf',
        '_pos_lf',
        '_force_rf',
        '_pos_rf',
        '_force_lr',
        '_pos_lr',
        '_force_rr',
        '_pos_rr',
        '_pos_body',
        '_vel_body',
        '_acc_imu',
        '_gyro_imu',
        '_fault',
    ]

    _fields_and_field_types = {
        'force_lf': 'float[3]',
        'pos_lf': 'float[3]',
        'force_rf': 'float[3]',
        'pos_rf': 'float[3]',
        'force_lr': 'float[3]',
        'pos_lr': 'float[3]',
        'force_rr': 'float[3]',
        'pos_rr': 'float[3]',
        'pos_body': 'float[3]',
        'vel_body': 'float[3]',
        'acc_imu': 'float[3]',
        'gyro_imu': 'float[3]',
        'fault': 'uint32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 3),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 3),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 3),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 3),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 3),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 3),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 3),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 3),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 3),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 3),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 3),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 3),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
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
        if 'pos_body' not in kwargs:
            self.pos_body = numpy.zeros(3, dtype=numpy.float32)
        else:
            self.pos_body = numpy.array(kwargs.get('pos_body'), dtype=numpy.float32)
            assert self.pos_body.shape == (3, )
        if 'vel_body' not in kwargs:
            self.vel_body = numpy.zeros(3, dtype=numpy.float32)
        else:
            self.vel_body = numpy.array(kwargs.get('vel_body'), dtype=numpy.float32)
            assert self.vel_body.shape == (3, )
        if 'acc_imu' not in kwargs:
            self.acc_imu = numpy.zeros(3, dtype=numpy.float32)
        else:
            self.acc_imu = numpy.array(kwargs.get('acc_imu'), dtype=numpy.float32)
            assert self.acc_imu.shape == (3, )
        if 'gyro_imu' not in kwargs:
            self.gyro_imu = numpy.zeros(3, dtype=numpy.float32)
        else:
            self.gyro_imu = numpy.array(kwargs.get('gyro_imu'), dtype=numpy.float32)
            assert self.gyro_imu.shape == (3, )
        self.fault = kwargs.get('fault', int())

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
        if all(self.force_lf != other.force_lf):
            return False
        if all(self.pos_lf != other.pos_lf):
            return False
        if all(self.force_rf != other.force_rf):
            return False
        if all(self.pos_rf != other.pos_rf):
            return False
        if all(self.force_lr != other.force_lr):
            return False
        if all(self.pos_lr != other.pos_lr):
            return False
        if all(self.force_rr != other.force_rr):
            return False
        if all(self.pos_rr != other.pos_rr):
            return False
        if all(self.pos_body != other.pos_body):
            return False
        if all(self.vel_body != other.vel_body):
            return False
        if all(self.acc_imu != other.acc_imu):
            return False
        if all(self.gyro_imu != other.gyro_imu):
            return False
        if self.fault != other.fault:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

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

    @builtins.property
    def pos_body(self):
        """Message field 'pos_body'."""
        return self._pos_body

    @pos_body.setter
    def pos_body(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'pos_body' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 3, \
                "The 'pos_body' numpy.ndarray() must have a size of 3"
            self._pos_body = value
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
                "The 'pos_body' field must be a set or sequence with length 3 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._pos_body = numpy.array(value, dtype=numpy.float32)

    @builtins.property
    def vel_body(self):
        """Message field 'vel_body'."""
        return self._vel_body

    @vel_body.setter
    def vel_body(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'vel_body' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 3, \
                "The 'vel_body' numpy.ndarray() must have a size of 3"
            self._vel_body = value
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
                "The 'vel_body' field must be a set or sequence with length 3 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._vel_body = numpy.array(value, dtype=numpy.float32)

    @builtins.property
    def acc_imu(self):
        """Message field 'acc_imu'."""
        return self._acc_imu

    @acc_imu.setter
    def acc_imu(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'acc_imu' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 3, \
                "The 'acc_imu' numpy.ndarray() must have a size of 3"
            self._acc_imu = value
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
                "The 'acc_imu' field must be a set or sequence with length 3 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._acc_imu = numpy.array(value, dtype=numpy.float32)

    @builtins.property
    def gyro_imu(self):
        """Message field 'gyro_imu'."""
        return self._gyro_imu

    @gyro_imu.setter
    def gyro_imu(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'gyro_imu' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 3, \
                "The 'gyro_imu' numpy.ndarray() must have a size of 3"
            self._gyro_imu = value
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
                "The 'gyro_imu' field must be a set or sequence with length 3 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._gyro_imu = numpy.array(value, dtype=numpy.float32)

    @builtins.property
    def fault(self):
        """Message field 'fault'."""
        return self._fault

    @fault.setter
    def fault(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'fault' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'fault' field must be an unsigned integer in [0, 4294967295]"
        self._fault = value
