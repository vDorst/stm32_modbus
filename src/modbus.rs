use core::default;
use defmt::Format;
use heapless::Vec;
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

pub struct Modbus {
    buf: Vec<u8, 256>,
    addr: u8,
    state: ModbusState,
    pub tmr: Option<SpawnHandle>,
}

impl Modbus {
    pub fn new(addr: u8) -> Self {
        Self {
            buf: Vec::new(),
            addr,
            state: ModbusState::default(),
            tmr: None,
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
        defmt::println!("MB: {:X} {}", data, self.state);
    }

    /// Should be called bij de t1.5/t3.5 timer interrupt
    pub fn is_valid(&mut self) -> bool {
        let (cancel, valid) = match self.state {
            ModbusState::InTrans | ModbusState::Nok => {
                self.buf.clear();
                (false, false)
            }
            ModbusState::Idle | ModbusState::WaitForT3_5 => (true, self.buf.len() >= 4),
        };

        self.tmr.take().map(|handle| {
            if cancel { 
                let _ = handle.cancel();
                None
            } else {
                handle.reschedule_after(1146.micros()).ok()
            }
        });

        valid
    }
}
