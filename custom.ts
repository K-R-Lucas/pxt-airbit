/**
 * Functions are mapped to blocks using various macros
 * in comments starting with %. The most important macro
 * is "block", and it specifies that a block should be
 * generated for an **exported** function.
 */

//% color="#2DD635" weight=100
namespace AirBit {
    interface Vector3 {
        x: number;
        y: number;
        z: number;
    }

    interface Gyro extends Vector3 {
        return_id: number,
        calibration: Vector3,
        delta: Vector3,
        roll: number,
        pitch: number,
        yaw: number
    }

    interface Acc extends Vector3 {
        roll: number,
        pitch: number,
        yaw: number
    }

    interface IMU {
        address: number,
        config: number,
        power_management: number;
        id: number,
        signal_reset: number,
        user_control: number,
        gyro_config: number,
        acc_config: number,
        bar_address: number
    }

    interface PCA {
        address: number,
        
    }

    let acc: Acc;
    let gyro: Gyro;
    let estop: boolean = false;
    const imu: IMU = {
        address: 0x68,
        config: 0x01,
        power_management: 0x6B,
        id: 0x75,
        signal_reset: 0x69,
        user_control: 0x6A,
        gyro_config: 0x1B,
        acc_config: 0x1D,
        bar_address: 0x63
    }

    //% block
    export function initialise() {
        // Reset IMU
        pins.i2cWriteNumber(
            imu.address,
            imu.power_management << 8 | 0x80,
            NumberFormat.UInt16BE, false
        );

        basic.pause(600);
        pins.i2cWriteNumber(
            imu.address, imu.id,
            NumberFormat.UInt8BE, true
        );

        acc = {
            x: 0,
            y: 0,
            z: 0,
            roll: 0,
            pitch: 0,
            yaw: 0
        }

        gyro = {
            return_id: pins.i2cReadNumber(imu.address, NumberFormat.Int16BE, false),
            x: 0,
            y: 0,
            z: 0,
            roll: 0,
            pitch: 0,
            yaw: 0
        }

        basic.clearScreen();

        if (gyro.return_id >> 8) {
            basic.showString("Gyro found");
        } else {
            basic.showString("Gyro not found", 50);
        }

        // Set clock
        pins.i2cWriteNumber(
            imu.address,
            imu.power_management << 8 | 0x01,
            NumberFormat.UInt16BE, false
        );

        // Set IMU to standby
        pins.i2cWriteNumber(
            imu.address,
            imu.signal_reset << 8 | 0x07,
            NumberFormat.UInt16BE, false
        );

        // Disable FIFO
        pins.i2cWriteNumber(
            imu.address,
            imu.user_control << 8 | 0x00,
            NumberFormat.UInt16BE, false
        );

        pins.i2cWriteNumber(
            imu.address,
            imu.user_control << 8 | 0x00,
            NumberFormat.UInt16BE, false
        );

        // Set gyro filter to 250Hz
        pins.i2cWriteNumber(
            imu.address,
            imu.config << 8 | 0x00,
            NumberFormat.UInt16BE, false
        );

        // Set acc filter to 10.2Hz
        pins.i2cWriteNumber(
            imu.address,
            imu.acc_config << 8 | 0x05,
            NumberFormat.UInt16BE, false
        );
    }

    //% block
    export function readSensors() {
        pins.i2cWriteNumber(
            imu.address, 0x43,
            NumberFormat.Int8LE, true
        );

        gyro.x = pins.i2cReadNumber(0x68, NumberFormat.Int16BE, true);
        gyro.y = pins.i2cReadNumber(0x68, NumberFormat.Int16BE, true);
        gyro.z = pins.i2cReadNumber(0x68, NumberFormat.Int16BE, false);

        pins.i2cWriteNumber(
            0x68, 0x3B,
            NumberFormat.Int8LE, true
        )

        acc.x = pins.i2cReadNumber(0x68, NumberFormat.Int16BE, true);
        acc.y = pins.i2cReadNumber(0x68, NumberFormat.Int16BE, true);
        acc.z = pins.i2cReadNumber(0x68, NumberFormat.Int16BE, false);
    }

    //% block
    export function stabilise() {

    }

    //% block
    export function connectToChannel(channel: number) {

    }

    //% block
    export function calibrate(channel: number) {
        drone_instance.calibrate();
    }

    //% block
    export function arm() {

    }

    //% block
    export function disarm() {

    }

    //% block
    export function setThrottle(amount: number) {

    }

    //% block
    export function setPitch(amount: number) {

    }

    //% block
    export function setRoll(amount: number) {

    }

    //% block
    export function setYaw(amount: number) {

    }

    //% block
    export function emergencyStop() {
        estop = true;
    }
}

namespace AirbitRemote {
    //% block
    export function connectToChannel(channel: number) {
        
    }
}