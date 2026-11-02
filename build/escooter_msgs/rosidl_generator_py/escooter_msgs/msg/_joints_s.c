// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from escooter_msgs:msg/Joints.idl
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
#include "escooter_msgs/msg/detail/joints__struct.h"
#include "escooter_msgs/msg/detail/joints__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool escooter_msgs__msg__joints__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[33];
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
    assert(strncmp("escooter_msgs.msg._joints.Joints", full_classname_dest, 32) == 0);
  }
  escooter_msgs__msg__Joints * ros_message = _ros_message;
  {  // handle_bar
    PyObject * field = PyObject_GetAttrString(_pymsg, "handle_bar");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->handle_bar = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // rotational_plate
    PyObject * field = PyObject_GetAttrString(_pymsg, "rotational_plate");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->rotational_plate = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // front_wheel
    PyObject * field = PyObject_GetAttrString(_pymsg, "front_wheel");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->front_wheel = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * escooter_msgs__msg__joints__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Joints */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("escooter_msgs.msg._joints");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Joints");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  escooter_msgs__msg__Joints * ros_message = (escooter_msgs__msg__Joints *)raw_ros_message;
  {  // handle_bar
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->handle_bar);
    {
      int rc = PyObject_SetAttrString(_pymessage, "handle_bar", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rotational_plate
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->rotational_plate);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rotational_plate", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // front_wheel
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->front_wheel);
    {
      int rc = PyObject_SetAttrString(_pymessage, "front_wheel", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
