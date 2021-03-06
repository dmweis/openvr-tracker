use crate::openvr_adaptor;
use nalgebra as na;
use serde::Serialize;
use std::usize;

#[derive(Debug, Serialize)]
pub struct TrackedObjects {
    ts: u128,
    trackers: Vec<VrDevice>,
}

impl TrackedObjects {
    pub fn new(ts: u128, trackers: Vec<VrDevice>) -> Self {
        Self { ts, trackers }
    }
}

#[derive(Debug, Eq, PartialEq, Copy, Clone, Serialize)]
pub enum VrDeviceClass {
    Controller,
    LeftController,
    RightController,
    Tracker,
    HMD,
    Sensor,
    Other,
}

impl VrDeviceClass {
    pub fn from_openvr_types(
        device_class: openvr::TrackedDeviceClass,
        controller: Option<openvr::TrackedControllerRole>,
    ) -> Self {
        match device_class {
            openvr::TrackedDeviceClass::HMD => VrDeviceClass::HMD,
            openvr::TrackedDeviceClass::Controller => {
                if let Some(role) = controller {
                    match role {
                        openvr::TrackedControllerRole::LeftHand => VrDeviceClass::LeftController,
                        openvr::TrackedControllerRole::RightHand => VrDeviceClass::RightController,
                    }
                } else {
                    VrDeviceClass::Controller
                }
            }
            openvr::TrackedDeviceClass::GenericTracker => VrDeviceClass::Tracker,
            openvr::TrackedDeviceClass::TrackingReference => VrDeviceClass::Sensor,
            _ => VrDeviceClass::Other,
        }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct VrDevice {
    id: usize,
    tracked: bool,
    seen: bool,
    position: na::Point3<f32>,
    rotation: na::UnitQuaternion<f32>,
    class: VrDeviceClass,
}

impl VrDevice {
    pub fn new(id: usize) -> Self {
        Self {
            id,
            tracked: false,
            seen: false,
            position: na::Point3::new(0., 0., 0.),
            rotation: na::UnitQuaternion::identity(),
            class: VrDeviceClass::Other,
        }
    }

    pub fn update(
        &mut self,
        tracked: bool,
        pose: &dyn openvr_adaptor::OpenVRPose,
        class: VrDeviceClass,
    ) {
        self.tracked = tracked;
        if self.tracked {
            self.seen = true;
        }
        self.position = pose.to_position();
        self.rotation = pose.to_rotation();
        self.class = class;
    }

    pub fn id(&self) -> usize {
        self.id
    }

    pub fn seen(&self) -> bool {
        self.seen
    }
}
