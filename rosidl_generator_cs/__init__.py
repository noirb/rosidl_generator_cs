# Copyright 2014-2018 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ast import literal_eval

from rosidl_cmake import generate_files
from rosidl_parser.definition import AbstractGenericString
from rosidl_parser.definition import AbstractNestedType
from rosidl_parser.definition import AbstractSequence
from rosidl_parser.definition import AbstractString
from rosidl_parser.definition import AbstractWString
from rosidl_parser.definition import Array
from rosidl_parser.definition import BasicType
from rosidl_parser.definition import BoundedSequence
from rosidl_parser.definition import FLOATING_POINT_TYPES
from rosidl_parser.definition import NamespacedType
from rosidl_parser.definition import UnboundedSequence

# If enabled, many base message types (e.g. vector3, point) will
# be replaced with Unity types (e.g. UnityEngine.Vector3)
USE_UNITY_TYPES = True

def generate_cs(generator_arguments_file):
    mapping = {
        'msg.cs.em': '%s.cs'
    }

    return generate_files(
        generator_arguments_file, mapping,
        post_process_callback=prefix_with_bom_if_necessary)


def prefix_with_bom_if_necessary(content):
    try:
        content.encode('ASCII')
    except UnicodeError:
        prefix = '\ufeff' + \
            '// NOLINT: This file starts with a BOM ' + \
            'since it contain non-ASCII characters\n'
        content = prefix + content
    return content


MSG_TYPE_TO_CS = {    'byte' : 'sbyte',
                     'octet' : 'sbyte',
                      'char' : 'byte',
                      'bool' : 'bool',
                   'boolean' : 'bool',
                     'uint8' : 'byte',
                      'int8' : 'sbyte',
                    'uint16' : 'System.UInt16',
                     'int16' : 'System.Int16',
                    'uint32' : 'System.UInt32',
                     'int32' : 'System.Int32',
                    'uint64' : 'System.UInt64',
                     'int64' : 'System.Int64',
                     'float' : 'float',
                   'float32' : 'float',
                   'float64' : 'double',
                    'double' : 'double',
                    'string' : 'string',
                      'time' : 'ROSBridgeLib.msg_helpers.Time',
                  'duration' : 'ROSBridgeLib.msg_helpers.Duration'}

# These substitutions are purely for convenience when working with data on the
# Unity side so we we don't have to martial everything all the time. The Unity
# types here are safe to use because they serialize to the same JSON as the msg
# types they replace.
MSG_TO_UNITY = {      'geometry_msgs.msg.Point' : 'UnityEngine.Vector3',
                    'geometry_msgs.msg.Vector3' : 'UnityEngine.Vector3',
                 'geometry_msgs.msg.Quaternion' : 'UnityEngine.Quaternion'}

def msg_type_only_to_cs(type_):
    """
    Convert a message type into the C# definition, ignoring array types.

    @param type_: The message type
    @type type_: rosidl_parser.Type
    """
    if isinstance(type_, AbstractNestedType):
        type_ = type_.value_type
    if isinstance(type_, BasicType):
        cs_type = MSG_TYPE_TO_CS[type_.typename]
    elif isinstance(type_, AbstractString):
        cs_type = MSG_TYPE_TO_CS['string']
    elif isinstance(type_, AbstractWString):
        cs_type = MSG_TYPE_TO_CS['wstring']
    elif isinstance(type_, NamespacedType):
        typename = '.'.join(type_.namespaced_name())
        cs_type = typename
    else:
        assert False, type_

    if USE_UNITY_TYPES and cs_type in MSG_TO_UNITY:
        cs_type = MSG_TO_UNITY[cs_type]
    
    return cs_type


def msg_type_to_cs(type_):
    """
    Convert a message type into the C# definition, along with the array type.

    @param type_: The message type
    @type type_: rosidl_parser.Type
    """
    cs_type = msg_type_only_to_cs(type_)

    if isinstance(type_, AbstractNestedType):
        if isinstance(type_, UnboundedSequence):
            return 'System.Collections.Generic.List<%s> '% (cs_type)
        elif isinstance(type_, BoundedSequence):
            return 'System.Collections.Generic.List<%s> '% (cs_type)
            #\
            #    ('rosidl_runtime_cpp::BoundedVector<%s, %u, typename std::allocator_traits' +
            #     '<ContainerAllocator>::template rebind_alloc<%s>>') % (cpp_type,
            #                                                            type_.maximum_size,
            #                                                            cpp_type)
        else:
            assert isinstance(type_, Array)
            return '%s[] ' % cs_type #, type_.size)
    else:
        return cs_type


def value_to_cs(type_, value):
    """
    Convert a python value into a string representing that value in C#.

    This is equivalent to primitive_value_to_cs but can process arrays values as well

    Warning this still processes only primitive types
    @param type_: a ROS IDL type
    @type type_: builtin.str
    @param value: the value to convert
    @type value: python builtin (bool, int, float, str or list)
    @returns: a string containing the C++ representation of the value
    """
    assert not isinstance(type_, NamespacedType), \
        "Could not convert non-primitive type '%s' to C#" % (type_)
    assert value is not None, "Value for type '%s' must not be None" % (type_)

    if not isinstance(type_, AbstractNestedType):
        return primitive_value_to_cs(type_, value)

    cs_values = []
    is_string_array = isinstance(type_.value_type, AbstractGenericString)
    for single_value in value:
        cs_value = primitive_value_to_cs(type_.value_type, single_value)
        if is_string_array:
            tmp_cs_value = '{%s}' % cs_value
        else:
            tmp_cs_value = cs_value
        cs_values.append(tmp_cs_value)
    cs_value = '{%s}' % ', '.join(cs_values)
    if len(cs_values) > 1 and not is_string_array:
        # Only wrap in a second set of {} if the array length is > 1.
        # This avoids "warning: braces around scalar initializer"
        cs_value = '{%s}' % cs_value
    return cs_value


def primitive_value_to_cs(type_, value):
    """
    Convert a python value into a string representing that value in C#.

    Warning: The value has to be a primitive and not a list
      (aka this function doesn't work for arrays)
    @param type_: a ROS IDL type
    @type type_: builtin.str
    @param value: the value to convert
    @type value: python builtin (bool, int, float or str)
    @returns: a string containing the C++ representation of the value
    """
    assert isinstance(type_, (BasicType, AbstractGenericString)), \
        "Could not convert non-primitive type '%s' to C#" % (type_)
    assert value is not None, "Value for type '%s' must not be None" % (type_)

    if isinstance(type_, AbstractString):
        return '"%s"' % escape_string(value)

    if isinstance(type_, AbstractWString):
        return 'u"%s"' % escape_wstring(value)

    if isinstance(type_, AbstractGenericString):
        return '"%s"' % escape_string(value)

    if type_.typename == 'boolean':
        return 'true' if value else 'false'

    if type_.typename in [
        'short', 'unsigned short',
        'char', 'wchar',
        'double', 'long double',
        'octet',
        'int8', 'uint8',
        'int16', 'uint16',
    ]:
        return str(value)

    if type_.typename == 'int32':
        # Handle edge case for INT32_MIN
        # Specifically, MSVC is not happy in this case
        if -2147483648 == value:
            return '({0} - 1)'.format(value + 1)
        return '%s' % value

    if type_.typename == 'uint32':
        return '%s' % value

    if type_.typename == 'int64':
        # Handle edge case for INT64_MIN
        # See https://en.cppreference.com/w/cpp/language/integer_literal
        if -9223372036854775808 == value:
            return '(%sL - 1)' % (value + 1)
        return '%sL' % value

    if type_.typename == 'uint64':
        return '%sUL' % value

    if type_.typename == 'float':
        return '%sf' % value

    if type_.typename == 'string':
        return '"%s"' % value

    assert False, "unknown primitive type '%s'" % type_.typename


def default_value_from_type(type_):
    if isinstance(type_, AbstractGenericString):
        return '""'
    elif isinstance(type_, BasicType) and type_.typename in FLOATING_POINT_TYPES:
        return '0.0f'
    elif isinstance(type_, BasicType) and type_.typename == 'boolean':
        return 'false'
    return 0


def escape_string(s):
    s = s.replace('\\', '\\\\')
    s = s.replace('"', '\\"')
    return s


def escape_wstring(s):
    return escape_string(s)

def array_initializer(type_):
    cs_type = msg_type_only_to_cs(type_)

    if isinstance(type_, UnboundedSequence) or isinstance(type_, BoundedSequence):
        return "new System.Collections.Generic.List<%s>()"%(cs_type)
    
    return "new %s[%s]"%(cs_type, type_.size)
