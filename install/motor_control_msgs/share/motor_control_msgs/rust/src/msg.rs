#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to motor_control_msgs__msg__DmInterface

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DmInterface {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub id: Vec<i64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub q: Vec<f32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub dq: Vec<f32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub kp: Vec<f32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub kd: Vec<f32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub tau: Vec<f32>,

}



impl Default for DmInterface {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::DmInterface::default())
  }
}

impl rosidl_runtime_rs::Message for DmInterface {
  type RmwMsg = super::msg::rmw::DmInterface;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        id: msg.id.into(),
        q: msg.q.into(),
        dq: msg.dq.into(),
        kp: msg.kp.into(),
        kd: msg.kd.into(),
        tau: msg.tau.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        id: msg.id.as_slice().into(),
        q: msg.q.as_slice().into(),
        dq: msg.dq.as_slice().into(),
        kp: msg.kp.as_slice().into(),
        kd: msg.kd.as_slice().into(),
        tau: msg.tau.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      id: msg.id
          .into_iter()
          .collect(),
      q: msg.q
          .into_iter()
          .collect(),
      dq: msg.dq
          .into_iter()
          .collect(),
      kp: msg.kp
          .into_iter()
          .collect(),
      kd: msg.kd
          .into_iter()
          .collect(),
      tau: msg.tau
          .into_iter()
          .collect(),
    }
  }
}


