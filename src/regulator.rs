/*
TODOs:
[] add check for cyclic call (e.g. every second and skip if called faster)
[] implement regulator logic (missing distance to target, outside temp)
[] implement protection mechanism (e.g. if vl_temp2 > 75C)
[] add manual override
*/

use crate::{
    homematic::DEVICES,
    i2c::{REMOTE_STOP_PUMP, REMOTE_VL_SOLL2},
};
use core::cell::RefCell;
use defmt::{info, warn};
use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
use embassy_time::Instant;

/// manual override of vl_soll2 and pump_onoff
/// if set to Some((vl_soll2, pump_onoff)), use these values instead
/// None means automatic regulation
//pub static MANUAL_VL_SOLL2:
pub struct ManualOverride {
    pub vl_soll2: Option<u8>,     // vl_soll2 in 0.5 degree C steps
    pub pump_onoff: Option<bool>, // true = pump on, false = pump off
    pub active_till: Instant,
}

pub static MANUAL_OVERRIDE: Mutex<CriticalSectionRawMutex, RefCell<ManualOverride>> =
    Mutex::new(RefCell::new(ManualOverride {
        vl_soll2: None,
        pump_onoff: None,
        active_till: Instant::MIN, // not active
    }));

/// Cyclic regulator tick function to be called from main loop
///
/// Determine the following regulator values:
/// - target temperature for hot water (REMOTE_VL_SOLL2)
/// - target temperature for heating circuit (REMOTE_WW_SOLL2)
/// - power level (0-100%/255) of fire (?)
///
/// Do this based on:
/// - valve positions of heating regulators (DEVICES.valve_position (0-1.0))
/// - distance to target temperatures (DEVICES.set_point_temp - DEVICES.valve_act_temp)
/// - outside temp
///
/// and a protection mechanism:
/// - if BOILER_STATE.vl_temp2 > 75C, turn on pump and set REMOTE_VL_SOLL2 to 20C
///
pub fn regulator_tick() {
    let now = Instant::now();

    // target values to be determined
    let mut pump_onoff = false;
    let mut vl_soll2 = 10 * 2u8; // default 10C

    // gather: min, max, avg valve positions
    if let Some((_valve_pos_min, valve_pos_max, valve_pos_avg)) = DEVICES.lock(|devices| {
        let mut min = f32::MAX;
        let mut max = f32::MIN;
        let mut sum = 0.0f32;
        let mut count = 0u32;

        let devices = devices.borrow();
        for device in devices.iter() {
            let valve_position = device.valve_position as f32; // TODO limit to 0..1.0
            if valve_position < min {
                min = valve_position;
            }
            if valve_position > max {
                max = valve_position;
            }
            sum += valve_position;
            count += 1;
        }
        if count > 0 {
            Some((min, max, sum / (count as f32)))
        } else {
            None
        }
    }) {
        // info!(
        //     "Regulator: valve positions min={} max={} avg={}",
        //     valve_pos_min, valve_pos_max, valve_pos_avg
        // );

        // simple logic: if any valve position > 0.8, set vl_soll2 to 50C...
        if valve_pos_max > 0.8 {
            vl_soll2 = 50 * 2;
            pump_onoff = true;
        } else if valve_pos_max > 0.5 {
            vl_soll2 = 45 * 2;
            pump_onoff = true;
        } else if valve_pos_max > 0.2 || valve_pos_avg > 0.1 {
            vl_soll2 = 40 * 2;
            pump_onoff = true;
        } else if valve_pos_max > 0.1 {
            vl_soll2 = 10 * 2; // off here but pump on
            pump_onoff = true;
        } else {
            // all valves closed (<= 0.1 / 10%)
            vl_soll2 = 10 * 2;
            pump_onoff = false;
        }
    } else {
        // no devices available
        warn!("Regulator: no devices available");
    }

    // manual override?
    MANUAL_OVERRIDE.lock(|ho| {
        let ho = ho.borrow();
        if now < ho.active_till {
            if let Some(or_vl_soll2) = ho.vl_soll2 {
                info!(
                    "Regulator: manual override active, setting vl_soll2 to {}C",
                    or_vl_soll2 as f32 / 2.0
                );
                vl_soll2 = or_vl_soll2;
            }
            if let Some(pump) = ho.pump_onoff {
                info!(
                    "Regulator: manual override active, setting pump_onoff to {}",
                    pump
                );
                pump_onoff = pump;
            }
        }
    });

    // set the values in REMOTE_VL_SOLL2 and REMOTE_STOP_PUMP
    if REMOTE_VL_SOLL2.swap(vl_soll2, core::sync::atomic::Ordering::Relaxed) != vl_soll2 {
        info!("Changed: REMOTE_VL_SOLL2: {}°C", vl_soll2 as f32 / 2.0);
    }
    let stop_pump = if pump_onoff { 0 } else { 1 };
    if REMOTE_STOP_PUMP.swap(stop_pump, core::sync::atomic::Ordering::Relaxed) != stop_pump {
        info!("Changed: REMOTE_STOP_PUMP: {}", stop_pump);
    }
}
