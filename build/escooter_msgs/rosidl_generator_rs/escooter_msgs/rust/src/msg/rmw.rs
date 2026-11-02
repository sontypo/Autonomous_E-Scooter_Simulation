#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "escooter_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__escooter_msgs__msg__RPY() -> *const std::ffi::c_void;
}

#[link(name = "escooter_msgs__rosidl_generator_c")]
extern "C" {
    fn escooter_msgs__msg__RPY__init(msg: *mut RPY) -> bool;
    fn escooter_msgs__msg__RPY__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RPY>, size: usize) -> bool;
    fn escooter_msgs__msg__RPY__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RPY>);
    fn escooter_msgs__msg__RPY__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RPY>, out_seq: *mut rosidl_runtime_rs::Sequence<RPY>) -> bool;
}

// Corresponds to escooter_msgs__msg__RPY
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RPY {

    // This member is not documented.
    #[allow(missing_docs)]
    pub roll: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pitch: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub yaw: f64,

}



impl Default for RPY {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !escooter_msgs__msg__RPY__init(&mut msg as *mut _) {
        panic!("Call to escooter_msgs__msg__RPY__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RPY {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { escooter_msgs__msg__RPY__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { escooter_msgs__msg__RPY__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { escooter_msgs__msg__RPY__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RPY {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RPY where Self: Sized {
  const TYPE_NAME: &'static str = "escooter_msgs/msg/RPY";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__escooter_msgs__msg__RPY() }
  }
}


#[link(name = "escooter_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__escooter_msgs__msg__Pose() -> *const std::ffi::c_void;
}

#[link(name = "escooter_msgs__rosidl_generator_c")]
extern "C" {
    fn escooter_msgs__msg__Pose__init(msg: *mut Pose) -> bool;
    fn escooter_msgs__msg__Pose__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Pose>, size: usize) -> bool;
    fn escooter_msgs__msg__Pose__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Pose>);
    fn escooter_msgs__msg__Pose__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Pose>, out_seq: *mut rosidl_runtime_rs::Sequence<Pose>) -> bool;
}

// Corresponds to escooter_msgs__msg__Pose
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Pose {

    // This member is not documented.
    #[allow(missing_docs)]
    pub position: geometry_msgs::msg::rmw::Point,


    // This member is not documented.
    #[allow(missing_docs)]
    pub orientation: super::super::msg::rmw::RPY,

}



impl Default for Pose {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !escooter_msgs__msg__Pose__init(&mut msg as *mut _) {
        panic!("Call to escooter_msgs__msg__Pose__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Pose {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { escooter_msgs__msg__Pose__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { escooter_msgs__msg__Pose__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { escooter_msgs__msg__Pose__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Pose {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Pose where Self: Sized {
  const TYPE_NAME: &'static str = "escooter_msgs/msg/Pose";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__escooter_msgs__msg__Pose() }
  }
}


#[link(name = "escooter_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__escooter_msgs__msg__Joints() -> *const std::ffi::c_void;
}

#[link(name = "escooter_msgs__rosidl_generator_c")]
extern "C" {
    fn escooter_msgs__msg__Joints__init(msg: *mut Joints) -> bool;
    fn escooter_msgs__msg__Joints__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Joints>, size: usize) -> bool;
    fn escooter_msgs__msg__Joints__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Joints>);
    fn escooter_msgs__msg__Joints__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Joints>, out_seq: *mut rosidl_runtime_rs::Sequence<Joints>) -> bool;
}

// Corresponds to escooter_msgs__msg__Joints
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Joints {

    // This member is not documented.
    #[allow(missing_docs)]
    pub handle_bar: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub rotational_plate: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub front_wheel: f64,

}



impl Default for Joints {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !escooter_msgs__msg__Joints__init(&mut msg as *mut _) {
        panic!("Call to escooter_msgs__msg__Joints__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Joints {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { escooter_msgs__msg__Joints__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { escooter_msgs__msg__Joints__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { escooter_msgs__msg__Joints__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Joints {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Joints where Self: Sized {
  const TYPE_NAME: &'static str = "escooter_msgs/msg/Joints";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__escooter_msgs__msg__Joints() }
  }
}


