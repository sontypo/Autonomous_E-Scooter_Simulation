#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to escooter_msgs__msg__RPY

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::RPY::default())
  }
}

impl rosidl_runtime_rs::Message for RPY {
  type RmwMsg = super::msg::rmw::RPY;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        roll: msg.roll,
        pitch: msg.pitch,
        yaw: msg.yaw,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      roll: msg.roll,
      pitch: msg.pitch,
      yaw: msg.yaw,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      roll: msg.roll,
      pitch: msg.pitch,
      yaw: msg.yaw,
    }
  }
}


// Corresponds to escooter_msgs__msg__Pose

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Pose {

    // This member is not documented.
    #[allow(missing_docs)]
    pub position: geometry_msgs::msg::Point,


    // This member is not documented.
    #[allow(missing_docs)]
    pub orientation: super::msg::RPY,

}



impl Default for Pose {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Pose::default())
  }
}

impl rosidl_runtime_rs::Message for Pose {
  type RmwMsg = super::msg::rmw::Pose;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        position: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(msg.position)).into_owned(),
        orientation: super::msg::RPY::into_rmw_message(std::borrow::Cow::Owned(msg.orientation)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        position: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(&msg.position)).into_owned(),
        orientation: super::msg::RPY::into_rmw_message(std::borrow::Cow::Borrowed(&msg.orientation)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      position: geometry_msgs::msg::Point::from_rmw_message(msg.position),
      orientation: super::msg::RPY::from_rmw_message(msg.orientation),
    }
  }
}


// Corresponds to escooter_msgs__msg__Joints

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Joints::default())
  }
}

impl rosidl_runtime_rs::Message for Joints {
  type RmwMsg = super::msg::rmw::Joints;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        handle_bar: msg.handle_bar,
        rotational_plate: msg.rotational_plate,
        front_wheel: msg.front_wheel,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      handle_bar: msg.handle_bar,
      rotational_plate: msg.rotational_plate,
      front_wheel: msg.front_wheel,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      handle_bar: msg.handle_bar,
      rotational_plate: msg.rotational_plate,
      front_wheel: msg.front_wheel,
    }
  }
}


