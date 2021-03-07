use crate::tracking_messages::*;
use anyhow::Result;
use nalgebra as na;
use std::{collections::HashMap, usize};

pub struct VrDeviceManager {
    devices: HashMap<usize, VrDevice>,
    /// Context needs to be kept around for interop reasons
    /// Otherwise you get a segfault
    #[allow(dead_code)]
    context: openvr::Context,
    openvr_system: openvr::System,
}

impl VrDeviceManager {
    pub fn new() -> Result<Self> {
        let context = unsafe { openvr::init(openvr::ApplicationType::Other) }?;
        let openvr_system = context.system()?;
        Ok(Self {
            devices: HashMap::new(),
            context,
            openvr_system,
        })
    }

    pub fn update(&mut self) {
        let poses = self
            .openvr_system
            .device_to_absolute_tracking_pose(openvr::TrackingUniverseOrigin::Standing, 0.0);
        for (index, pose) in poses.iter().enumerate() {
            let device_entry = self
                .devices
                .entry(index)
                .or_insert_with(|| VrDevice::new(index));
            let tracked = pose.pose_is_valid();
            let device_class = self.openvr_system.tracked_device_class(index as u32);
            let controller_class = self
                .openvr_system
                .get_controller_role_for_tracked_device_index(index as u32);
            let class = VrDeviceClass::from_openvr_types(device_class, controller_class);
            let pose = pose.device_to_absolute_tracking();
            device_entry.update(tracked, pose, class);
        }
    }

    pub fn device_list(&self) -> Vec<VrDevice> {
        // super inefficient. But do we really care? It's only 64 elements
        let mut devices: Vec<_> = self.devices.values().cloned().collect();
        devices.sort_by_key(|item| item.id());
        devices
    }
}

pub trait OpenVRPose {
    fn to_position(&self) -> na::Point3<f32>;
    fn to_rotation(&self) -> na::UnitQuaternion<f32>;
}

impl OpenVRPose for [[f32; 4]; 3] {
    /// based on [Valve implementation on github](
    /// https://github.com/ValveSoftware/openvr/blob/60eb187801956ad277f1cae6680e3a410ee0873b/samples/unity_teleport_sample/Assets/SteamVR/Scripts/SteamVR_Utils.cs#L155)
    fn to_position(&self) -> na::Point3<f32> {
        let x = self[0][3];
        let y = self[1][3];
        let z = self[2][3];
        na::Point3::new(x, y, z)
    }

    /// Calculated rotation form pose matrix
    ///
    /// # Reference
    ///
    /// based on [Valve implementation on github](
    /// https://github.com/ValveSoftware/openvr/blob/60eb187801956ad277f1cae6680e3a410ee0873b/samples/unity_teleport_sample/Assets/SteamVR/Scripts/SteamVR_Utils.cs#L142)
    #[allow(clippy::many_single_char_names)]
    fn to_rotation(&self) -> na::UnitQuaternion<f32> {
        let m = self;
        let w = 0_f32.max(1. + m[0][0] + m[1][1] + m[2][2]).sqrt() / 2.0;
        let i = 0_f32.max(1. + m[0][0] - m[1][1] - m[2][2]).sqrt() / 2.0;
        let j = 0_f32.max(1. - m[0][0] + m[1][1] - m[2][2]).sqrt() / 2.0;
        let k = 0_f32.max(1. - m[0][0] - m[1][1] + m[2][2]).sqrt() / 2.0;
        let i = i.copysign(m[2][1] - m[1][2]);
        let j = j.copysign(m[0][2] - m[2][0]);
        let k = k.copysign(m[1][0] - m[0][1]);
        na::UnitQuaternion::from_quaternion(na::Quaternion::new(w, i, j, k))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_matrix_layout() {
        let matrix = [[0., 1., 2., 3.], [4., 5., 6., 7.], [8., 9., 10., 11.]];
        let position = matrix.to_position();
        assert_eq!(position.x as i32, 11);
        assert_eq!(position.y as i32, 3);
        assert_eq!(position.z as i32, 7);
    }
}
