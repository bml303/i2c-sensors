use i2c_linux::{    
    I2c, Message, ReadFlags, WriteFlags,
};
use std::fs::File;
use std::path::Path;
use std::{thread, time};

pub fn get_bus(bus_path: &Path)  -> Result<I2c<File>, std::io::Error> {
    I2c::from_path(bus_path)
}

pub fn set_slave(i2c: &mut I2c<File>, dev_addr: u16) -> Result<(), std::io::Error> {
    i2c.smbus_set_slave_address(dev_addr, false)
} 

pub fn read_20_bits(i2c: &mut I2c<File>, register: u8) -> Result<i32, std::io::Error> {
    let mut val: [u8; 3] = [0, 0, 0];
    let _bytes_read = i2c.i2c_read_block_data(register, &mut val)?;
    let val: i32 = (val[2] >> 4) as i32 + ((val[1] as i32) << 4) + ((val[0] as i32) << 12);
    Ok(val)
}

pub fn read_word(i2c: &mut I2c<File>, register: u8) -> Result<u16, std::io::Error> {
    i2c.smbus_read_word_data(register)
}

pub fn read_byte(i2c: &mut I2c<File>, register: u8) -> Result<u8, std::io::Error> {
    i2c.smbus_read_byte_data(register)
}

pub fn read_bytes(i2c: &mut I2c<File>, device_addr: u16, data: &mut [u8]) -> Result<(), std::io::Error> {
    let read_message = Message::Read { address: device_addr, data: data, flags: ReadFlags::empty() };
    let mut messages = [read_message];
    i2c.i2c_transfer(&mut messages)
}

pub fn write_byte_single(i2c: &mut I2c<File>, data: u8) -> Result<(), std::io::Error> {
    i2c.smbus_write_byte(data)
}

pub fn write_byte(i2c: &mut I2c<File>, register: u8, data: u8) -> Result<(), std::io::Error> {
    i2c.smbus_write_byte_data(register, data)
}

pub fn write_bytes<const LEN: usize>(i2c: &mut I2c<File>, device_addr: u16, data: [u8; LEN]) -> Result<(), std::io::Error> {
    let data = &data;
    let write_message = Message::Write { address: device_addr, data: data, flags: WriteFlags::empty() };
    let mut messages = [write_message];
    i2c.i2c_transfer(&mut messages)
}

pub fn write_word(i2c: &mut I2c<File>, register: u8, data: u16) -> Result<(), std::io::Error> {
    i2c.smbus_write_word_data(register, data)
}

pub fn delay(milli_secs: u32) {    
    let delay = time::Duration::from_millis(milli_secs as u64);
    thread::sleep(delay);
}

