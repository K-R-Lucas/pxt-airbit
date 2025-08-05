enum ControlSignals {
    throttle = 't',
    pitch = 'p',
    roll = 'r',
    yaw = 'y',
    arm = 'a',
    disarm = 'd',
    estop = 'e'
}

interface Motors {
    M0: number,
    M1: number,
    M2: number,
    M3: number
}

interface Angles {
    pitch: number,
    roll: number,
    yaw: number
}

interface Controls extends Angles {
    throttle: number
}

interface Safety {
    estop: boolean,
    crashed: boolean,
    armed: boolean,
    low_battery: boolean
}

class Vector3 {
    x = 0;
    y = 0;
    z = 0;

    constructor(x: number, y: number, z: number) {
        x = x;
        y = y;
        z = z;
    }

    add(v: Vector3) {
        return new Vector3(
            this.x + v.x, this.y + v.y, this.z + v.z
        );
    }

    sub(v: Vector3) {
        return new Vector3(
            this.x - v.x, this.y - v.x, this.z - v.z
        );
    }

    mul(v: Vector3) {
        return new Vector3(
            this.x*v.x, this.y*v.y, this.z*v.z
        );
    }

    scale(v: number) {
        return new Vector3(
            this.x*v, this.y*v, this.z*v
        );
    }

    div(v: Vector3) {
        return new Vector3(
            this.x/v.x, this.y/v.y, this.z/v.z
        );
    }

    iAdd(v: Vector3) {
        this.x += v.x;
        this.y += v.y;
        this.z += v.z;
    }

    iSub(v: Vector3) {
        this.x -= v.x;
        this.y -= v.y;
        this.z -= v.z;
    }

    iMul(v: Vector3) {
        this.x *= v.x;
        this.y *= v.y;
        this.z *= v.z;
    }

    iScale(v: number) {
        this.x *= v;
        this.y *= v;
        this.z *= v;
    }

    iDiv(v: Vector3) {
        this.x /= v.x;
        this.y /= v.y;
        this.z /= v.z;
    }

    copy() {
        return new Vector3(this.x, this.y, this.z);
    }

    set(v: Vector3) {
        this.x = v.x;
        this.y = v.y;
        this.z = v.z;
    }
}

class Accelerometer {
    reading: Vector3 = new Vector3(0, 0, 0);
    reference: Angles = {
        pitch: 0,
        roll: 0,
        yaw: 0
    };
    angles: Angles = {
        pitch: 0,
        roll: 0,
        yaw: 0
    };

    config_register: number = 29;
    imu: IMU;

    constructor(imu: IMU) {
        this.imu = imu;
    }

    init() {
        this.imu.writeRegister(this.config_register, 0x03, false);
    }

    read() {
        pins.i2cWriteNumber(
            this.imu.address, 0x3B,
            NumberFormat.Int8LE, true
        );

        this.reading.x = pins.i2cReadNumber(this.imu.address, NumberFormat.Int16BE, true);
        this.reading.y = pins.i2cReadNumber(this.imu.address, NumberFormat.Int16BE, true);
        this.reading.z = pins.i2cReadNumber(this.imu.address, NumberFormat.Int16BE, false);
    }
};

class Gyroscope {
    angles: Angles = {
        pitch: 0,
        roll: 0,
        yaw: 0
    };
    reading: Vector3 = new Vector3(0, 0, 0);
    delta: Vector3 = new Vector3(0, 0, 0);
    reference: Vector3 = new Vector3(0, 0, 0);
    id: number = 0;

    imu: IMU;

    constructor(imu: IMU) {
        this.imu = imu;
    }

    init() {
        pins.i2cWriteNumber(
            this.imu.address, this.imu.who_am_i,
            NumberFormat.UInt8BE, true
        );

        this.id = pins.i2cReadNumber(this.imu.address, NumberFormat.Int16BE, false) >> 8;

        if (this.id <= 0) {
            throw new Error("Gyroscope not found!");
        }

        this.imu.writeRegister(this.imu.config, 0x00, false);
    }

    read() {
        pins.i2cWriteNumber(
            this.imu.address, 0x43,
            NumberFormat.Int8LE, true
        );

        this.reading.x = pins.i2cReadNumber(this.imu.address, NumberFormat.Int16BE, true);
        this.reading.y = pins.i2cReadNumber(this.imu.address, NumberFormat.Int16BE, true);
        this.reading.z = pins.i2cReadNumber(this.imu.address, NumberFormat.Int16BE, false);
    }
};

class Timer {
    dt: number = 0;
    t0: number = 0;
    t1: number = 0;

    constructor() {
        this.t0 = input.runningTimeMicros();
        this.t1 = this.t0;
    }

    get_delta() {
        this.t1 = input.runningTimeMicros();
        this.dt = this.t1 - this.t0;
        this.t0 = this.t1;
        return this.dt;
    }
}

class PID {
    angles: Angles = {
        pitch: 0,
        roll: 0,
        yaw: 0
    };

    delta: Angles = {
        pitch: 0,
        roll: 0,
        yaw: 0
    };

    last: Angles = {
        pitch: 0,
        roll: 0,
        yaw: 0
    };

    integral: Angles = {
        pitch: 0,
        roll: 0,
        yaw: 0
    };

    correction: Angles = {
        pitch: 0,
        roll: 0,
        yaw: 0
    };

    integral_range: number = 5;
    integral_limit: number = 4;
    yaw_limit: number = 50;
    roll_pitch_p: number = 0.7;
    roll_pitch_i: number = 0.004;
    roll_pitch_d: number = 15;
    yaw_p: number = 4;
    yaw_d: number = 10;

    constructor() {}

    update(drone: Drone) {
        this.angles.pitch = drone.controls.pitch - drone.imu.gyroscope.angles.pitch;
        this.angles.roll = drone.controls.roll - drone.imu.gyroscope.angles.roll;
        this.angles.yaw = drone.controls.yaw - drone.imu.gyroscope.angles.yaw;

        this.delta.pitch = this.angles.pitch - this.last.pitch;
        this.delta.roll = this.angles.roll - this.last.roll;
        this.delta.yaw = this.angles.yaw - this.last.yaw;

        this.last.pitch = this.angles.pitch;
        this.last.roll = this.angles.roll;
        this.last.yaw = this.angles.yaw;

        if (drone.controls.throttle > 50) {
            if (Math.abs(this.angles.pitch) < this.integral_range) {
                this.integral.pitch += this.angles.pitch;
            }

            if (Math.abs(this.angles.roll) < this.integral_range) {
                this.integral.roll += this.angles.roll;
            }
        }

        this.correction.pitch = Math.min(
            Math.max(this.integral.pitch*this.roll_pitch_i, -this.integral_limit), this.integral_limit
        );

        this.correction.roll = Math.min(
            Math.max(this.integral.roll*this.roll_pitch_i, -this.integral_limit), this.integral_limit
        );

        this.correction.pitch += this.angles.pitch*this.roll_pitch_p + this.delta.pitch*this.roll_pitch_d;
        this.correction.roll += this.angles.roll*this.roll_pitch_p + this.delta.roll*this.roll_pitch_d;
        this.correction.yaw = Math.min(
            Math.max(this.angles.yaw*this.yaw_p + this.delta.yaw*this.yaw_d, -this.yaw_limit), this.yaw_limit
        );

        let throttle_scaled = 2.55*drone.controls.throttle;

        drone.motors.M0 = Math.min(
            Math.max(
                Math.round(throttle_scaled + this.correction.roll + this.correction.pitch + this.correction.yaw), 0
            ), 255
        );

        drone.motors.M1 = Math.min(
            Math.max(
                Math.round(throttle_scaled + this.correction.roll - this.correction.pitch - this.correction.yaw), 0
            ), 255
        );

        drone.motors.M2 = Math.min(
            Math.max(
                Math.round(throttle_scaled - this.correction.roll + this.correction.pitch - this.correction.yaw), 0
            ), 255
        );

        drone.motors.M3 = Math.min(
            Math.max(
                Math.round(throttle_scaled - this.correction.roll - this.correction.pitch + this.correction.yaw), 0
            ), 255
        );
    }
}

class IMU {
    id = 0x75;
    config: number = 0x01;
    address: number = 0x68;
    gyro_config: number = 0x1B;
    power_mgmt: number = 0x6B;
    bar_address: number = 0x63;
    signal_reset: number = 0x69;
    user_control: number = 0x6A;
    accel_config: number = 0x1D;
    who_am_i: number = 0x75;

    accelerometer: Accelerometer = new Accelerometer(this);
    gyroscope: Gyroscope = new Gyroscope(this);

    constructor() {
        this.reset();
        this.gyroscope.init();
        this.accelerometer.init()
    }

    writeRegister(register: number, value: number, repeat: boolean = false) {
        pins.i2cWriteNumber(this.address, register << 8 | value, NumberFormat.UInt16BE, repeat);
    }

    reset() {
        this.writeRegister(this.power_mgmt, 0x80);
        pins.i2cWriteNumber(
            this.address,
            this.power_mgmt << 8 | 0x80,
            NumberFormat.UInt16BE, false
        );
        basic.pause(500);

        pins.i2cWriteNumber(
            this.address, this.power_mgmt << 8 | 0x01,
            NumberFormat.UInt16BE, false
        );

        pins.i2cWriteNumber(
            this.address, this.signal_reset << 8 | 0x07,
            NumberFormat.UInt16BE, false
        );

        pins.i2cWriteNumber(
            this.address, this.user_control << 8 | 0x00,
            NumberFormat.UInt16BE, false
        );
    }
}

class PCA {
    address = 98;
    pwm0 = 2;
    pwm1 = 3;
    pwm2 = 4;
    pwm3 = 5;

    constructor() {}

    writeRegister(register: number, value: number) {
        pins.i2cWriteNumber(
            this.address, register << 8 | value,
            NumberFormat.UInt16BE, false
        );
    }

    writeMotors(motors: Motors) {
        this.writeRegister(
            this.pwm0, motors.M0
        );

        this.writeRegister(
            this.pwm1, motors.M1
        );

        this.writeRegister(
            this.pwm2, motors.M2
        );

        this.writeRegister(
            this.pwm3, motors.M3
        );
    }
}

class Drone {
        throttle: number = 0;
        motors: Motors = {
            M0: 0,
            M1: 0,
            M2: 0,
            M3: 0
        };

        voltage: number = 0.0;
        init_state: boolean = false;

        safety: Safety = {
            estop: false,
            crashed: false,
            armed: false,
            low_battery: false
        }

        controls: Controls = {
            throttle: 0,
            pitch: 0,
            roll: 0,
            yaw: 0
        };

        physics_timer: Timer = new Timer();
        pid: PID;
        imu: IMU;
        pca: PCA;

        battery_scale_factor: number = 0.00594;
        battery_voltage_smooth: number = 0;
        battery_voltage: number = 0;
        battery_level: number = 0;

        constructor() {
            i2crr.setI2CPins(DigitalPin.P2, DigitalPin.P1);
            this.pid = new PID();
            this.imu = new IMU();
            this.pca = new PCA();
            this.init_state = true;
        }

        arm() {
            this.imu.reset();
            this.safety.armed = true;
        }

        disarm() {
            this.motors = {
                M0: 0,
                M1: 0,
                M2: 0,
                M3: 0
            }

            this.pca.writeMotors(this.motors);
            this.imu.reset();

            this.safety.armed = false;
        }

        estop() {
            this.safety.estop = true;
            this.disarm();
        }

        checkBatteryLevel() {
            this.safety.low_battery = (this.battery_level <= 0);
        }

        checkDroneCrashed() {
            this.safety.crashed = (Math.abs(this.imu.gyroscope.angles.roll) > 90) || (Math.abs(this.imu.gyroscope.angles.pitch) > 90);

            if (this.safety.crashed) {
                this.estop();
            }
        }

        update(stabilise: boolean = true) {
            if (this.safety.estop || !this.safety.armed) return;

            this.calculateBatteryVoltage();
            this.smoothBatteryVoltage();
            this.calculateBatteryLevel();
            this.checkBatteryLevel();
            if (this.safety.low_battery) return;

            this.imu.accelerometer.read();
            this.imu.gyroscope.read();
            this.calculateAngles();
            this.checkDroneCrashed();
            if (this.safety.crashed) return;

            if (stabilise) {
                this.pid.update(this);
            }

            this.pca.writeMotors(this.motors);
        }

        calculateAngles() {
            let dt = this.physics_timer.get_delta();
            this.imu.accelerometer.angles.pitch = -57.295*Math.atan2(this.imu.accelerometer.reading.y, this.imu.accelerometer.reading.z) - this.imu.accelerometer.reference.pitch;
            this.imu.accelerometer.angles.roll = -57.295*Math.atan2(this.imu.accelerometer.reading.x, this.imu.accelerometer.reading.z) - this.imu.accelerometer.reference.roll;

            let temp = -0.00762939*dt;
            this.imu.gyroscope.delta.set(this.imu.gyroscope.reading);
            this.imu.gyroscope.delta.iSub(this.imu.gyroscope.reference);
            this.imu.gyroscope.delta.iScale(temp);
            this.imu.gyroscope.delta.y *= -1;

            this.imu.gyroscope.angles.roll = 0.01*this.imu.accelerometer.angles.roll + 0.99*(this.imu.gyroscope.delta.y + this.imu.gyroscope.angles.roll);
            this.imu.gyroscope.angles.pitch = 0.01*this.imu.accelerometer.angles.pitch + 0.99*(this.imu.gyroscope.delta.x + this.imu.gyroscope.angles.pitch);
        }

        calculateBatteryVoltage() {
            this.battery_voltage = pins.analogReadPin(AnalogPin.P0) * this.battery_scale_factor;
        }

        smoothBatteryVoltage() {
            this.battery_voltage_smooth = 0.1*this.battery_voltage + 0.9*this.battery_voltage_smooth;
        }

        calculateBatteryLevel() {
            this.battery_level = (this.battery_voltage_smooth - 3.4)/0.8;
        }
    };

namespace AirBit {
    let drone: Drone;
    
    //% block="init()"
    //% group="Setup"
    export function init() {
        drone = new Drone();
    }

    //% block="arm()"
    //% group="Safety"
    export function arm() {
        drone.arm();
    }

    //% block="disarm()"
    //% group="Safety"
    export function disarm() {
        drone.disarm();
    }

    //% block="emergencyStop()"
    //% group="Safety"
    export function emergencyStop() {
        drone.estop();
        drone = undefined;
    }

    //% block="getGyroPitch()"
    //% group="Telemetry"
    export function getGyroPitch(): number {
        return drone.imu.gyroscope.angles.pitch;
    }

    //% block="getGyroRoll()"
    //% group="Telemetry"
    export function getGyroRoll(): number {
        return drone.imu.gyroscope.angles.roll;
    }

    //% block="getGyroYaw()"
    //% group="Telemetry"
    export function getGyroYaw(): number {
        return drone.imu.gyroscope.angles.yaw;
    }

    //% block="getAccelPitch()"
    //% group="Telemetry"
    export function getAccelPitch(): number {
        return drone.imu.accelerometer.angles.pitch;
    }

    //% block="getAccelRoll()"
    //% group="Telemetry"
    export function getAccelRoll(): number {
        return drone.imu.accelerometer.angles.roll;
    }

    //% block="getAccelYaw()"
    //% group="Telemetry"
    export function getAccelYaw(): number {
        return drone.imu.accelerometer.angles.yaw;
    }

    //% block="isArmed()"
    //% group="State"
    export function isArmed(): boolean {
        return drone.safety.armed;
    }

    //% block="hasCrashed()"
    //% group="State"
    export function hasCrashed(): boolean {
        return drone.safety.crashed;
    }

    //% block="isEmergencyStopped()"
    //% group="State"
    export function isEmergencyStopped(): boolean {
        return drone.safety.estop;
    }

    //% block="update(stabilise$stabilise)"
    //% group="Control"
    //% stabilise.defl=true
    export function update(stabilise: boolean) {
        drone.update(stabilise);
    }

    //% block="setWiFiChannel($channel)"
    //% group="Control"
    export function setWifiChannel(channel: number) {
        radio.setGroup(channel);
    }

    //% block="isInitialised()"
    //% group="State"
    export function isInitialised(): boolean {
        return (drone !== undefined) && drone.init_state;
    }

    //% draggableParameters="reporter"
    //% block="onControlReceived($signal $value)"
    //% group="Control"
    export function onControlReceived(signal: ControlSignals, handler: (value: number) => void) {
        radio.onReceivedValue(
            function (name: string, value: number) {
                if (name == signal) {
                    handler(value);
                }
            }
        );
    }
};