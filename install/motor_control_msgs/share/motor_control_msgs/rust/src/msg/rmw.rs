#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "motor_control_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__motor_control_msgs__msg__DmInterface() -> *const std::ffi::c_void;
}

#[link(name = "motor_control_msgs__rosidl_generator_c")]
extern "C" {
    fn motor_control_msgs__msg__DmInterface__init(msg: *mut DmInterface) -> bool;
    fn motor_control_msgs__msg__DmInterface__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DmInterface>, size: usize) -> bool;
    fn motor_control_msgs__msg__DmInterface__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DmInterface>);
    fn motor_control_msgs__msg__DmInterface__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DmInterface>, out_seq: *mut rosidl_runtime_rs::Sequence<DmInterface>) -> bool;
}

// Corresponds to motor_control_msgs__msg__DmInterface
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DmInterface {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub id: rosidl_runtime_rs::Sequence<i64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub q: rosidl_runtime_rs::Sequence<f32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub dq: rosidl_runtime_rs::Sequence<f32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub kp: rosidl_runtime_rs::Sequence<f32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub kd: rosidl_runtime_rs::Sequence<f32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub tau: rosidl_runtime_rs::Sequence<f32>,

}



impl Default for DmInterface {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !motor_control_msgs__msg__DmInterface__init(&mut msg as *mut _) {
        panic!("Call to motor_control_msgs__msg__DmInterface__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DmInterface {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { motor_control_msgs__msg__DmInterface__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { motor_control_msgs__msg__DmInterface__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { motor_control_msgs__msg__DmInterface__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DmInterface {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DmInterface where Self: Sized {
  const TYPE_NAME: &'static str = "motor_control_msgs/msg/DmInterface";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__motor_control_msgs__msg__DmInterface() }
  }
}


