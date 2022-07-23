// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from two_wheel_control_msgs:msg/Command.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "two_wheel_control_msgs/msg/detail/command__struct.h"
#include "two_wheel_control_msgs/msg/detail/command__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool two_wheel_control_msgs__msg__command__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[44];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("two_wheel_control_msgs.msg._command.Command", full_classname_dest, 43) == 0);
  }
  two_wheel_control_msgs__msg__Command * ros_message = _ros_message;
  {  // p
    PyObject * field = PyObject_GetAttrString(_pymsg, "p");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->p = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // thetay
    PyObject * field = PyObject_GetAttrString(_pymsg, "thetay");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->thetay = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // thetaz
    PyObject * field = PyObject_GetAttrString(_pymsg, "thetaz");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->thetaz = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // p_dot
    PyObject * field = PyObject_GetAttrString(_pymsg, "p_dot");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->p_dot = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // thetay_dot
    PyObject * field = PyObject_GetAttrString(_pymsg, "thetay_dot");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->thetay_dot = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // thetaz_dot
    PyObject * field = PyObject_GetAttrString(_pymsg, "thetaz_dot");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->thetaz_dot = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // p_ddot
    PyObject * field = PyObject_GetAttrString(_pymsg, "p_ddot");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->p_ddot = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // thetay_ddot
    PyObject * field = PyObject_GetAttrString(_pymsg, "thetay_ddot");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->thetay_ddot = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // thetaz_ddot
    PyObject * field = PyObject_GetAttrString(_pymsg, "thetaz_ddot");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->thetaz_ddot = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * two_wheel_control_msgs__msg__command__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Command */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("two_wheel_control_msgs.msg._command");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Command");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  two_wheel_control_msgs__msg__Command * ros_message = (two_wheel_control_msgs__msg__Command *)raw_ros_message;
  {  // p
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->p);
    {
      int rc = PyObject_SetAttrString(_pymessage, "p", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // thetay
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->thetay);
    {
      int rc = PyObject_SetAttrString(_pymessage, "thetay", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // thetaz
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->thetaz);
    {
      int rc = PyObject_SetAttrString(_pymessage, "thetaz", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // p_dot
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->p_dot);
    {
      int rc = PyObject_SetAttrString(_pymessage, "p_dot", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // thetay_dot
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->thetay_dot);
    {
      int rc = PyObject_SetAttrString(_pymessage, "thetay_dot", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // thetaz_dot
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->thetaz_dot);
    {
      int rc = PyObject_SetAttrString(_pymessage, "thetaz_dot", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // p_ddot
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->p_ddot);
    {
      int rc = PyObject_SetAttrString(_pymessage, "p_ddot", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // thetay_ddot
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->thetay_ddot);
    {
      int rc = PyObject_SetAttrString(_pymessage, "thetay_ddot", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // thetaz_ddot
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->thetaz_ddot);
    {
      int rc = PyObject_SetAttrString(_pymessage, "thetaz_ddot", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
