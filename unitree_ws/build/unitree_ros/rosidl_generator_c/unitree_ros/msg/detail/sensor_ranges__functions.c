// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from unitree_ros:msg/SensorRanges.idl
// generated code does not contain a copyright notice
#include "unitree_ros/msg/detail/sensor_ranges__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
unitree_ros__msg__SensorRanges__init(unitree_ros__msg__SensorRanges * msg)
{
  if (!msg) {
    return false;
  }
  // front
  // left
  // right
  return true;
}

void
unitree_ros__msg__SensorRanges__fini(unitree_ros__msg__SensorRanges * msg)
{
  if (!msg) {
    return;
  }
  // front
  // left
  // right
}

bool
unitree_ros__msg__SensorRanges__are_equal(const unitree_ros__msg__SensorRanges * lhs, const unitree_ros__msg__SensorRanges * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // front
  if (lhs->front != rhs->front) {
    return false;
  }
  // left
  if (lhs->left != rhs->left) {
    return false;
  }
  // right
  if (lhs->right != rhs->right) {
    return false;
  }
  return true;
}

bool
unitree_ros__msg__SensorRanges__copy(
  const unitree_ros__msg__SensorRanges * input,
  unitree_ros__msg__SensorRanges * output)
{
  if (!input || !output) {
    return false;
  }
  // front
  output->front = input->front;
  // left
  output->left = input->left;
  // right
  output->right = input->right;
  return true;
}

unitree_ros__msg__SensorRanges *
unitree_ros__msg__SensorRanges__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_ros__msg__SensorRanges * msg = (unitree_ros__msg__SensorRanges *)allocator.allocate(sizeof(unitree_ros__msg__SensorRanges), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(unitree_ros__msg__SensorRanges));
  bool success = unitree_ros__msg__SensorRanges__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
unitree_ros__msg__SensorRanges__destroy(unitree_ros__msg__SensorRanges * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    unitree_ros__msg__SensorRanges__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
unitree_ros__msg__SensorRanges__Sequence__init(unitree_ros__msg__SensorRanges__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_ros__msg__SensorRanges * data = NULL;

  if (size) {
    data = (unitree_ros__msg__SensorRanges *)allocator.zero_allocate(size, sizeof(unitree_ros__msg__SensorRanges), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = unitree_ros__msg__SensorRanges__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        unitree_ros__msg__SensorRanges__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
unitree_ros__msg__SensorRanges__Sequence__fini(unitree_ros__msg__SensorRanges__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      unitree_ros__msg__SensorRanges__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

unitree_ros__msg__SensorRanges__Sequence *
unitree_ros__msg__SensorRanges__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_ros__msg__SensorRanges__Sequence * array = (unitree_ros__msg__SensorRanges__Sequence *)allocator.allocate(sizeof(unitree_ros__msg__SensorRanges__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = unitree_ros__msg__SensorRanges__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
unitree_ros__msg__SensorRanges__Sequence__destroy(unitree_ros__msg__SensorRanges__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    unitree_ros__msg__SensorRanges__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
unitree_ros__msg__SensorRanges__Sequence__are_equal(const unitree_ros__msg__SensorRanges__Sequence * lhs, const unitree_ros__msg__SensorRanges__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!unitree_ros__msg__SensorRanges__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
unitree_ros__msg__SensorRanges__Sequence__copy(
  const unitree_ros__msg__SensorRanges__Sequence * input,
  unitree_ros__msg__SensorRanges__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(unitree_ros__msg__SensorRanges);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    unitree_ros__msg__SensorRanges * data =
      (unitree_ros__msg__SensorRanges *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!unitree_ros__msg__SensorRanges__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          unitree_ros__msg__SensorRanges__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!unitree_ros__msg__SensorRanges__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
