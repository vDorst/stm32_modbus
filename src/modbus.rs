use core::default;
use defmt::Format;
use heapless::pool::singleton::{Pool, Box};
use heapless::{Vec, spsc::Producer};
use systick_monotonic::fugit::ExtU64;

#[derive(Debug, Default, Format)]
pub enum ModbusState {
    #[default]
    Idle,
    InTrans,
    Nok,
    WaitForT3_5,
}

use crate::app::__rtic_internal_tmr_modbus_MyMono_SpawnHandle as SpawnHandle;
use crate::{ModbusSerBuf, SERMB};

pub struct Modbus {
    buf: Vec<u8, 256>,
    addr: u8,
    state: ModbusState,
    pub tmr: Option<SpawnHandle>,
    queue: Producer<'static, Box<SERMB>, 4>,
}

pub fn calc_crc(data: &[u8]) -> u16 {
    let mut crc = 0xffffu16;
    for &byte in data {
        crc ^= u16::from(byte);
        for _ in 0..8 {
            let check = crc & 0x0001;
            crc >>= 1;
            if check != 0 {
                crc ^= 0xA001;
            }
        }
    }

    crc
}

impl Modbus {
    pub fn new(addr: u8, queue: Producer<'static, Box<SERMB>, 4>) -> Self {
        Self {
            buf: Vec::new(),
            addr,
            state: ModbusState::default(),
            tmr: None,
            queue,
        }
    }

    /// Should be called by the UART RX interrupt
    pub fn char_recv(&mut self, data: u8) {
        match self.state {
            ModbusState::Idle => {
                if !(data == self.addr || data == 0) {
                    self.state = ModbusState::Nok;
                } else {
                    self.state = ModbusState::InTrans;
                }
            }
            ModbusState::InTrans | ModbusState::Nok => (),
            ModbusState::WaitForT3_5 => self.state = ModbusState::Nok,
        }

        if matches!(self.state, ModbusState::InTrans) {
            if self.buf.push(data).is_err() {
                self.state = ModbusState::Nok;
            }
        }
        // defmt::println!("MB: {:X} {}", data, self.state);
    }

    /// Should be called bij de t1.5/t3.5 timer interrupt
    pub fn timer_handle(&mut self) {
        let (back_to_idle, valid) = match self.state {
            ModbusState::InTrans => (false, false),
            ModbusState::Nok => (false, false),
            ModbusState::Idle => (true, false),
            ModbusState::WaitForT3_5 => (true, self.buf.len() >= 4),
        };

        if back_to_idle {
            
            self.tmr.take().map(|tmr| tmr.cancel());
            if valid {
                let crc_start = self.buf.len() - 2;
                let crc_data = &self.buf[0..crc_start];
                let crc_calc = calc_crc(crc_data).to_le_bytes();
                let crc_calc = crc_calc.as_slice();
                let crc_recv = &self.buf[crc_start..];
                if crc_calc == crc_recv {
                    if let Some(buf) = SERMB::alloc() {
                        let mut d = buf.init(Vec::new());
                        d.clone_from(&self.buf);
                        let _ = self.queue.enqueue(d);
                    }
                } else {
                    defmt::println!("CRC C: {:X} R {:X} D: {:X}", crc_calc, crc_recv, crc_data);
                }
            }
            self.buf.clear();
            self.state = ModbusState::Idle;
        } else {
            self.state = ModbusState::WaitForT3_5;
            self.tmr = crate::app::tmr_modbus::spawn_after(ExtU64::millis(2)).ok();
        }

        //defmt::println!("MB_TMR: {} V: {} T: {}", self.state, valid, self.tmr.is_some());
    }

    pub fn reset(&mut self) {
        self.buf.clear();
        self.state = ModbusState::Idle;
    }
}
