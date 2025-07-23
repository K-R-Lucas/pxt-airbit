//% color="#07c3aa" weight=100
namespace AirBit {
    interface Vector3 {
        x: number,
        y: number,
        z: number
    }

    interface EulerAngles {
        roll: number,
        pitch: number,
        yaw: number
    }

    interface Gyro extends Vector3, EulerAngles {
        return_id: number,
        calibration: Vector3,
        delta: Vector3
    }

    interface Acc extends Vector3, EulerAngles {
        calibration: Vector3,
        pitch_offset: number,
        roll_offset: number
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
        leduot: number,
        mode1: number,
        mode2: number,
        pwm0: number,
        pwm1: number,
        pwm2: number,
        pwm3: number,
        mode2_config: number,
        return_id: number
    }

    interface Timer {
        t0: number,
        t1: number,
        dt: number,
        radio_t0: number,
        radio_t1: number,
        radio_dt: number
    }

    interface PID extends EulerAngles {
        delta: EulerAngles,
        last: EulerAngles,
        correction: EulerAngles,
        integral: EulerAngles,
        integral_range: number,
        integral_limit: number,
        yaw_correction_limit: number,
    }

    interface Motors {
        M0: number,
        M1: number,
        M2: number,
        M3: number
    }

    interface Drone extends EulerAngles {
        throttle: number,
        throttleScaled: number,
        motors: Motors,
        battery_voltage: number
    }

    interface PIDSettings {
        roll_pitch_p: number,
        roll_pitch_i: number,
        roll_pitch_d: number,
        yaw_p: number,
        yaw_d: number
    }

    interface Settings {
        pid: PIDSettings
    }

    interface Safety {
        estop: boolean,
        armed: boolean,
        low_battery: boolean,
        tilted: boolean
    }

    interface Bar {
        address: number,
        return_id: number
    }
    
    let safety: Safety = {
        estop: false,
        armed: false,
        low_battery: false,
        tilted: false
    }

    let drone: Drone = {
        throttle: 0,
        throttleScaled: 0,
        motors: {
            M0: 0,
            M1: 0,
            M2: 0,
            M3: 0
        },
        battery_voltage: 0,
        roll: 0,
        pitch: 0,
        yaw: 0
    };

    let pca: PCA = {
        address: 0x62,
        leduot: 0x08,
        mode1: 0x00,
        mode2: 0x01,
        pwm0: 0x02,
        pwm1: 0x03,
        pwm2: 0x04,
        pwm3: 0x05,
        mode2_config: 0x05,
        return_id: 0x00
    }
    
    let acc: Acc = {
        x: 0,
        y: 0,
        z: 0,
        roll: 0,
        pitch: 0,
        yaw: 0,
        calibration: {
            x: 0,
            y: 0,
            z: 0
        },
        pitch_offset: 0,
        roll_offset: 0
    };

    let gyro: Gyro = {
        return_id: 0,
        x: 0,
        y: 0,
        z: 0,
        roll: 0,
        pitch: 0,
        yaw: 0,
        calibration: {
            x: 0,
            y: 0,
            z: 0
        },
        delta: {
            x: 0,
            y: 0,
            z: 0
        }
    };

    let timer: Timer = {
        t0: 0,
        t1: 0,
        dt: 0,
        radio_t0: 0,
        radio_t1: 0,
        radio_dt: 0
    };

    let pid: PID = {
        delta: {
            roll: 0,
            pitch: 0,
            yaw: 0
        },

        last: {
            roll: 0,
            pitch: 0,
            yaw: 0
        },

        correction: {
            roll: 0,
            pitch: 0,
            yaw: 0
        },

        integral: {
            roll: 0,
            pitch: 0,
            yaw: 0
        },

        roll: 0,
        pitch: 0,
        yaw: 0,

        integral_range: 5,
        integral_limit: 4,
        yaw_correction_limit: 50
    };

    let settings: Settings = {
        pid: {
            roll_pitch_p: 0.7,
            roll_pitch_i: 0.004,
            roll_pitch_d: 15,
            yaw_p: 4,
            yaw_d: 10
        }
    };

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

    let bar: Bar = {
        address: 0x63,
        return_id: 0
    }

    function checkCommunication() {
        timer.radio_t1 = input.runningTime();
        timer.radio_dt = timer.radio_t1 - timer.radio_t0;
        timer.radio_t0 = timer.radio_t1;

        if (timer.radio_dt >= 5000) {
            disarm();
        }
    }

    function watchSafety() {
        readTelemetry(true, true, true, true);

        if ((Math.abs(gyro.roll) >= 90) || (Math.abs(gyro.pitch) >= 90)) {
            disarm();
        }
    }

    //% block
    //& group="Setup"
    export function initialise() {
        if (safety.estop) return;

        i2crr.setI2CPins(DigitalPin.P2, DigitalPin.P1);
        
        pid = {
            delta: {
                roll: 0,
                pitch: 0,
                yaw: 0
            },

            last: {
                roll: 0,
                pitch: 0,
                yaw: 0
            },

            correction: {
                roll: 0,
                pitch: 0,
                yaw: 0
            },

            integral: {
                roll: 0,
                pitch: 0,
                yaw: 0
            },

            roll: 0,
            pitch: 0,
            yaw: 0,

            integral_range: 5,
            integral_limit: 4,
            yaw_correction_limit: 50
        }

        let t = input.runningTime();
        timer = {
            t0: t,
            t1: t,
            dt: 0,
            radio_t0: t,
            radio_t1: t,
            radio_dt: 0
        }

        drone = {
            throttle: 0,
            throttleScaled: 0,
            motors: {
                M0: 0,
                M1: 0,
                M2: 0,
                M3: 0
            },
            battery_voltage: 0,
            roll: 0,
            pitch: 0,
            yaw: 0
        }

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
            yaw: 0,
            calibration: {
                x: 0,
                y: 0,
                z: 0
            },
            pitch_offset: 0,
            roll_offset: 0
        }

        gyro = {
            return_id: pins.i2cReadNumber(imu.address, NumberFormat.Int16BE, false),
            x: 0,
            y: 0,
            z: 0,
            roll: 0,
            pitch: 0,
            yaw: 0,
            calibration: {
                x: 0,
                y: 0,
                z: 0
            },
            delta: {
                x: 0,
                y: 0,
                z: 0
            }
        }

        basic.clearScreen();

        if (gyro.return_id >> 8) {
            basic.showString("Gyroscope found!");
        } else {
            basic.showString("No gyroscope found!", 50);
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

        bar = {
            address: 0x63,
            return_id: 0
        };

        /// Start the barometer
        pins.i2cWriteNumber(
            bar.address, 0x805D,
            NumberFormat.UInt16BE, true
        );

        pins.i2cWriteNumber(
            bar.address, 0xEFC8,
            NumberFormat.UInt16BE, true
        );

        bar.return_id = pins.i2cReadNumber(
            bar.address, NumberFormat.UInt16LE, true
        );

        if (bar.return_id) {
            basic.showString("Barometer found!");
        } else {
            basic.showString("No barometer found!");
        }
        
        radio.onDataReceived(checkCommunication)
        basic.forever(watchSafety);

        radio.onReceivedString(
            function (in_string) {
                if (safety.estop) return;
                
                if (in_string == "e") {
                    safety.estop = true;
                }

                switch (in_string) {
                    case 'e':
                        safety.estop = true;
                        break;
                    
                    case 'a':
                        arm();
                        break;

                    case 'd':
                        disarm();
                        break;
                }
            }
        )
    }

    //% block
    //& group="Setup"
    export function arm() {
        if (safety.estop) return;

        safety.armed = true;

        initialise();

        writePca(pca.mode1, 0x80);
        writePca(pca.mode2, pca.mode2_config);
        writePca(pca.leduot, 0xAA);

        setMotorSpeeds(0, 0, 0, 0);

        pins.i2cWriteNumber(
            pca.address, pca.mode2,
            NumberFormat.UInt8BE, true
        )

        pca.return_id = pins.i2cReadNumber(pca.address, NumberFormat.UInt8BE, false);

        basic.clearScreen();
        if (pca.return_id) {
            basic.showString("PCA found!");
        } else {
            basic.showString("PCA not found!", 50);
        }
    }

    //% block
    //& group="Setup"
    export function disarm() {
        safety.armed = false;

        setMotorSpeeds(0, 0, 0, 0);

        writePca(pca.mode1, 0x80);
        writePca(pca.mode2, pca.mode2_config);
        writePca(pca.leduot, 0xAA);
    }

    //% block="Read Telemetry (gyro, accel, battery, angles) $read_gyro $read_acc $read_battery $calculate_angles"
    //% inlineInputMode=inline
    //& group="Data"
    export function readTelemetry(read_gyro: boolean = true, read_acc: boolean = true, read_battery: boolean = true, calculate_angles: boolean = false) {
        if (read_gyro) {
            pins.i2cWriteNumber(
                imu.address, 0x43,
                NumberFormat.Int8LE, true
            );

            gyro.x = pins.i2cReadNumber(0x68, NumberFormat.Int16BE, true);
            gyro.y = pins.i2cReadNumber(0x68, NumberFormat.Int16BE, true);
            gyro.z = pins.i2cReadNumber(0x68, NumberFormat.Int16BE, false);
        }

        if (read_acc) {
            pins.i2cWriteNumber(
                0x68, 0x3B,
                NumberFormat.Int8LE, true
            );

            acc.x = pins.i2cReadNumber(0x68, NumberFormat.Int16BE, true);
            acc.y = pins.i2cReadNumber(0x68, NumberFormat.Int16BE, true);
            acc.z = pins.i2cReadNumber(0x68, NumberFormat.Int16BE, false);
        }

        if (read_battery) {
            drone.battery_voltage = pins.analogReadPin(AnalogPin.P0) * 0.00594;
        }

        if (calculate_angles) {
            timer.t1 = input.runningTime();
            timer.dt = timer.t1 - timer.t0;
            timer.t0 = timer.t1;

            acc.pitch = -57.295*Math.atan2(acc.y, acc.z) - acc.pitch_offset;
            acc.roll = -57.295*Math.atan2(acc.x, acc.z) -acc.roll_offset;
            
            let temp = -0.00000762939*timer.dt;
            gyro.delta.x = temp*(gyro.x - gyro.calibration.x);
            gyro.delta.y = -temp*(gyro.y - gyro.calibration.y);
            gyro.delta.z = temp*(gyro.z - gyro.calibration.z);

            gyro.roll = 0.01*acc.roll + 0.99*(gyro.delta.y + gyro.roll);
            gyro.pitch = 0.01*acc.pitch + 0.99*(gyro.delta.x + gyro.pitch);
            gyro.yaw += gyro.delta.y;
        }
    }

    //% block="Motor Speed(m0 $m0 m1 $m1 m2 $m2 m3 $m3"
    //% inlineInputMode=inline
    //& group="Control"
    export function setMotorSpeeds(m0: number, m1: number, m2: number, m3: number) {
        if (safety.estop || !safety.armed) return

        pins.i2cWriteNumber(
            pca.address,
            pca.pwm0 << 8 | m3,
            NumberFormat.UInt16BE,
            false
        )
        pins.i2cWriteNumber(
            pca.address,
            pca.pwm1 << 8 | m2,
            NumberFormat.UInt16BE,
            false
        )
        pins.i2cWriteNumber(
            pca.address,
            pca.pwm2 << 8 | m1,
            NumberFormat.UInt16BE,
            false
        )
        pins.i2cWriteNumber(
            pca.address,
            pca.pwm3 << 8 | m0,
            NumberFormat.UInt16BE,
            false
        )
    }

    function writePca(register: number, value: number) {
        if (safety.estop || !safety.armed)
        pins.i2cWriteNumber(
            pca.address,
            register << 8 | value,
            NumberFormat.UInt16BE,
            false
        )
    }

    //% block
    //& group="Control"
    export function stabilise() {
        if (safety.estop || !safety.armed) return

        pid.roll = drone.roll - gyro.roll;
        pid.pitch = drone.pitch - gyro.pitch;
        pid.yaw = drone.yaw - gyro.yaw;

        pid.delta.roll = pid.roll - pid.last.roll;
        pid.delta.pitch = pid.pitch - pid.last.pitch;
        pid.delta.yaw = pid.yaw - pid.last.yaw;

        pid.last.roll = pid.roll;
        pid.last.pitch = pid.pitch;
        pid.last.yaw = pid.yaw;

        if (drone.throttle > 50) {
            if (Math.abs(pid.roll) < pid.integral_range) {
                pid.integral.roll += pid.roll;
            }

            if (Math.abs(pid.pitch) < pid.integral_range) {
                pid.integral.pitch += pid.pitch;
            }
        }

        pid.correction.roll = Math.min(
            Math.max(pid.integral.roll*settings.pid.roll_pitch_i, -pid.integral_limit), pid.integral_limit
        );

        pid.correction.pitch = Math.min(
            Math.max(pid.integral.pitch*settings.pid.roll_pitch_i, -pid.integral_limit), pid.integral_limit
        );

        pid.correction.roll += pid.roll*settings.pid.roll_pitch_p + pid.delta.roll*settings.pid.roll_pitch_d;
        pid.correction.pitch += pid.pitch*settings.pid.roll_pitch_p + pid.delta.pitch*settings.pid.roll_pitch_d;
        pid.correction.yaw = Math.min(
            Math.max(pid.yaw*settings.pid.yaw_p + pid.delta.yaw*settings.pid.yaw_d, -pid.yaw_correction_limit), pid.yaw_correction_limit
        );

        drone.throttleScaled = 2.55*drone.throttle;

        drone.motors.M0 = Math.min(
            Math.max(
                Math.round(drone.throttleScaled + pid.correction.roll + pid.correction.pitch + pid.correction.yaw), 0
            ), 255
        );

        drone.motors.M1 = Math.min(
            Math.max(
                Math.round(drone.throttleScaled + pid.correction.roll - pid.correction.pitch - pid.correction.yaw), 0
            ), 255
        );

        drone.motors.M2 = Math.min(
            Math.max(
                Math.round(drone.throttleScaled - pid.correction.roll + pid.correction.pitch + pid.correction.yaw), 0
            ), 255
        );

        drone.motors.M3 = Math.min(
            Math.max(
                Math.round(drone.throttleScaled - pid.correction.roll - pid.correction.pitch + pid.correction.yaw), 0
            ), 255
        );

        setMotorSpeeds(drone.motors.M0, drone.motors.M1, drone.motors.M2, drone.motors.M3);
    }

    //% block
    //& group="Setup"
    export function connectToChannel(channel: number) {
        radio.setGroup(channel);
    }

    //% block
    //& group="Setup"
    export function calibrate(samples: number = 100) {
        if (safety.estop) return;

        gyro.calibration.x = 0;
        gyro.calibration.y = 0;
        gyro.calibration.z = 0;

        for (let i = 0; i < samples; i++) {
            readTelemetry(true, true, false, false);
            gyro.calibration.x += gyro.x;
            gyro.calibration.y += gyro.y;
            gyro.calibration.z += gyro.z;

            acc.calibration.x += acc.x;
            acc.calibration.y += acc.y;
            acc.calibration.z += acc.z;
            basic.pause(5);
        }

        gyro.calibration.x /= samples;
        gyro.calibration.y /= samples;
        gyro.calibration.z /= samples;

        acc.calibration.x /= samples;
        acc.calibration.y /= samples;
        acc.calibration.z /= samples;
        
        acc.pitch_offset = -57.295 * Math.atan2(acc.y, acc.z);
        acc.roll_offset = -57.295 * Math.atan2(acc.x, acc.z);
    }

    //% block
    //& group="Control"
    export function setThrottle(amount: number) {
        drone.throttle = Math.min(Math.max(amount, 0), 100);
    }

    //% block
    //& group="Control"
    export function setPitch(amount: number) {
        drone.pitch = Math.min(Math.max(amount, -45), 45);
    }

    //% block
    //& group="Control"
    export function setRoll(amount: number) {
        drone.roll = Math.min(Math.max(amount, -45), 45);
    }

    //% block
    //& group="Control"
    export function setYaw(amount: number) {
        drone.yaw = amount;
    }
}