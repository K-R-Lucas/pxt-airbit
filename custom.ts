enum ControlSignals {
    estop = 0,
    pitch = 1,
    armed = 2,
    roll = 3,
    throttle = 4,
    yaw = 5
}

enum MotorLabels {
    M0 = 0,
    M1 = 1,
    M2 = 2,
    M3 = 3
};

enum PIDAxis {
    rollPitch = 0,
    yaw = 1
};

const controlSignalMap: Array<string> = [
    'e', 'p', 'a', 'r', 't', 'y'
];

interface Position {
    x: number,
    y: number
};

const ledBinPositions: Array<Position> = [
    {x: 0, y: 0},
    {x: 1, y: 0},
    {x: 1, y: 1},
    {x: 2, y: 1},
    {x: 2, y: 2},
    {x: 3, y: 2},
    {x: 3, y: 3},
    {x: 4, y: 3},
];

interface Motors {
    M0: number,
    M1: number,
    M2: number,
    M3: number
};

interface Angles {
    pitch: number,
    roll: number,
    yaw: number
};

interface Controls extends Angles {
    throttle: number
};

interface Safety {
    estop: boolean,
    crashed: boolean,
    armed: boolean,
    low_battery: boolean
};

interface Handler {
    name: string,
    func: (value?: number | Boolean) => void
};

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
};

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

    config2: number = 0x1D;
    imu: IMU;

    constructor(imu: IMU) {
        this.imu = imu;
    }

    init() {
        this.imu.writeRegister(this.config2, 0b00000011, false);
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
    config: number = 0x1B;

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
        basic.showString("G");
        }

        this.imu.writeRegister(this.config, 0b00000000, false);
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

    calibrate(samples: number = 100) {
        basic.showIcon(IconNames.Asleep);
        this.reference.x = 0;
        this.reference.y = 0;
        this.reference.z = 0;

        for (let i = 0; i < samples; i++) {
            this.read();

            this.reference.x += this.reading.x;
            this.reference.y += this.reading.y;
            this.reference.z += this.reading.z;
            
            basic.pause(5);
        }

        this.reference.x /= samples;
        this.reference.y /= samples;
        this.reference.z /= samples;
        
        this.imu.accelerometer.reference.pitch = -57.295 * Math.atan2(this.imu.accelerometer.reading.y, this.imu.accelerometer.reading.z);
        this.imu.accelerometer.reference.roll = -57.295 * Math.atan2(this.imu.accelerometer.reading.x, this.imu.accelerometer.reading.z);

        basic.showIcon(IconNames.Happy);
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
};

class PID {
    active: boolean = true;

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
    yaw_limit: number = 30;
    roll_pitch_p: number = 1.5; // defl 0.9
    roll_pitch_i: number = 0.1;
    roll_pitch_d: number = 70;
    yaw_p: number = 50; // defl 5
    yaw_d: number = 70;

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
        let en = (throttle_scaled > 0) ? 1 : 0;

        drone.motors.M0 = en * Math.min(
            Math.max(
                Math.round(throttle_scaled + this.correction.roll + this.correction.pitch + this.correction.yaw), 0
            ), 255
        );

        drone.motors.M1 = en * Math.min(
            Math.max(
                Math.round(throttle_scaled + this.correction.roll - this.correction.pitch - this.correction.yaw), 0
            ), 255
        );

        drone.motors.M2 = en * Math.min(
            Math.max(
                Math.round(throttle_scaled - this.correction.roll + this.correction.pitch - this.correction.yaw), 0
            ), 255
        );

        drone.motors.M3 = en * Math.min(
            Math.max(
                Math.round(throttle_scaled - this.correction.roll - this.correction.pitch + this.correction.yaw), 0
            ), 255
        );
    }
};

class IMU {
    id = 0x75;
    config: number = 0x1A;
    address: number = 0x68;
    power_mgmt: number = 0x6B;
    bar_address: number = 0x63;
    signal_reset: number = 0x68;
    user_control: number = 0x6A;
    who_am_i: number = 0x75;

    accelerometer: Accelerometer;
    gyroscope: Gyroscope;

    constructor() {
        this.reset();
        basic.pause(500);

        this.gyroscope = new Gyroscope(this);
        this.accelerometer = new Accelerometer(this);
        this.gyroscope.init();
        this.accelerometer.init()
    }

    writeRegister(register: number, value: number, repeat: boolean = false) {
        pins.i2cWriteNumber(this.address, register << 8 | value, NumberFormat.UInt16BE, repeat);
    }

    reset() {
        this.writeRegister(this.power_mgmt,   0b10000000);
        basic.pause(500);

        this.writeRegister(this.config,       0b00000000);
        this.writeRegister(this.power_mgmt,   0b00000001);
        this.writeRegister(this.signal_reset, 0b00000011);
        this.writeRegister(this.user_control, 0b00000001);
    }
};

class PCA {
    address: number = 98;
    pwm0: number = 2;
    pwm1: number = 3;
    pwm2: number = 4;
    pwm3: number = 5;

    mode1: number = 0;
    mode2: number = 1;
    ledout: number = 8;

    mode1_config: number = 128;
    mode2_config: number = 5;

    id: number = 0;

    constructor() {
        this.writeRegister(this.mode1, this.mode1_config);
        this.writeRegister(this.mode2, this.mode2_config);
        this.writeRegister(this.ledout, 170);
        this.writeMotors({M0: 0, M1: 0, M2: 0, M3: 0});

        pins.i2cWriteNumber(this.address, this.mode2, NumberFormat.UInt8BE, true);
        this.id = pins.i2cReadNumber(this.address, NumberFormat.UInt8BE, false);

        if (!this.id) {
            basic.showString("M");
        }
    }

    writeRegister(register: number, value: number, repeat: boolean = false) {
        pins.i2cWriteNumber(
            this.address, register << 8 | value,
            NumberFormat.UInt16BE, repeat
        );
    }

    writeMotors(motors: Motors) {
        this.writeRegister(
            this.pwm0, motors.M3
        );

        this.writeRegister(
            this.pwm1, motors.M2
        );

        this.writeRegister(
            this.pwm2, motors.M1
        );

        this.writeRegister(
            this.pwm3, motors.M0
        );
    }
};

function as_uint32(v: number): number {
    let buf = pins.createBuffer(4);
    buf.setNumber(NumberFormat.Float32BE, 0, v);
    return buf.getNumber(NumberFormat.UInt32BE, 0);
}

function as_float32(v: number): number {
    let buf = pins.createBuffer(4);
    buf.setNumber(NumberFormat.UInt32BE, 0, v);
    return buf.getNumber(NumberFormat.Float32BE, 0);
}

function f16_to_f32(x: number): number {
    let e = Math.trunc((x & 0x7C00) >> 10);
    let m = Math.trunc((x & 0x03FF) << 13);
    let v = Math.trunc(as_uint32(x) >> 23);

    return as_float32(
        (x & 0x8000) << 16 | (+(e != 0)) * ((e + 112) << 23 | m)
                           | ((+(e == 0)) & (+(m != 0))) * (
                                (v - 32) << 23 | ((m << (150 - v)) & 0x007FE000)
                           ));
}

function f32_to_f16(x: number): number {
    let b = Math.trunc(as_uint32(x) + 0x00001000);
    let e = Math.trunc((b & 0x7F800000) >> 23);
    let m = Math.trunc(b & 0x007FFFFF);

    return (b & 0x80000000) >> 16 | (+(e > 112)) * ((((e - 112) << 10) & 0x7C00) | m >> 13)
                                  | ((+(e < 113)) & (+(e > 101))) * ((((0x007FF000 + m) >> (125 - e)) + 1) >> 1)
                                  | (+(e > 143)) * 0x7FFF;

}

class State {
    throttle: number = 0;
    pitch: number = 0;
    roll: number = 0;
    yaw: number = 0;
    armed: Boolean = false;
    estop: Boolean = false;
    crashed: Boolean = false;
    charging: Boolean = false;
    charged: Boolean = false;
    low_battery: Boolean = false;
    drone: Drone;
    acc: Vector3;

    packet: Buffer;

    constructor(drone: Drone) {
        this.packet = Buffer.create(15);
        this.acc = new Vector3(0, 0, 0);

        this.drone = drone;
    }

    getPacket() {
        this.packet.setUint8(0,
            +this.drone.safety.armed
            | +this.drone.safety.estop << 1
            | +this.drone.safety.crashed << 2
            | +this.drone.charging << 3
            | +this.drone.charged << 4
            | +this.drone.safety.low_battery << 5
        );

        this.packet.setNumber(NumberFormat.UInt16BE, 1, f32_to_f16(this.drone.battery_voltage));
        this.packet.setNumber(NumberFormat.UInt16BE, 3, f32_to_f16(this.drone.imu.gyroscope.angles.pitch));
        this.packet.setNumber(NumberFormat.UInt16BE, 5, f32_to_f16(this.drone.imu.gyroscope.angles.roll));
        this.packet.setNumber(NumberFormat.UInt16BE, 7, f32_to_f16(this.drone.imu.gyroscope.angles.yaw));
        this.packet.setNumber(NumberFormat.UInt16BE, 9, f32_to_f16(this.drone.imu.accelerometer.reading.x));
        this.packet.setNumber(NumberFormat.UInt16BE, 11, f32_to_f16(this.drone.imu.accelerometer.reading.y));
        this.packet.setNumber(NumberFormat.UInt16BE, 13, f32_to_f16(this.drone.imu.accelerometer.reading.z));
    }

    loadPacket(packet: Buffer) {
        let flags = packet.getUint8(0);
        this.armed = !!(flags & 0b00000001);
        this.estop = !!(flags & 0b00000010);

        this.throttle = f16_to_f32(packet.getNumber(NumberFormat.UInt16BE, 1));
        this.pitch =    f16_to_f32(packet.getNumber(NumberFormat.UInt16BE, 3));
        this.roll =     f16_to_f32(packet.getNumber(NumberFormat.UInt16BE, 5));
        this.yaw =      f16_to_f32(packet.getNumber(NumberFormat.UInt16BE, 7));

        for (let handler of this.drone.handlers) {
            switch (handler.name) {
                case controlSignalMap[ControlSignals.estop]:
                    handler.func(this.estop);
                    break;
                
                case controlSignalMap[ControlSignals.pitch]:
                    handler.func(this.pitch);
                    break;
            
                case controlSignalMap[ControlSignals.armed]:
                    handler.func(this.armed);
                    break;
            
                case controlSignalMap[ControlSignals.roll]:
                    handler.func(this.roll);
                    break;
            
                case controlSignalMap[ControlSignals.throttle]:
                    handler.func(this.throttle);
                    break;
            
                case controlSignalMap[ControlSignals.yaw]:
                    handler.func(this.yaw);
                    break;
            }
        }
    }

    clear() {
        this.pitch = 0;
        this.armed = false;
        this.roll = 0;
        this.throttle = 0;
        this.yaw = 0;
        this.estop = false;
    }
};

class Drone {
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

    controls: State;

    physics_timer: Timer = new Timer();
    pid: PID;
    imu: IMU;
    pca: PCA;

    battery_scale_factor: number = 0.00594;
    battery_voltage_smooth: number = 0;
    battery_voltage: number = 0;
    battery_level: number = 0;
    charging: boolean = false;
    charged: boolean = true;
    A0: number = 0;

    handlers: Array<Handler>;

    constructor() {
        i2crr.setI2CPins(DigitalPin.P2, DigitalPin.P1);
        this.controls = new State(this);
        this.pid = new PID();
        this.imu = new IMU();
        this.pca = new PCA();
        this.handlers = [];
        this.init_state = true;
    }

    arm() {
        this.controls.clear();

        this.imu.reset();
        this.safety.crashed = false;
        this.safety.armed = true;
    }

    disarm() {
        this.safety.armed = false;

        this.pca.writeMotors({M0: 0, M1: 0, M2: 0, M3: 0});
        this.imu.reset();

        this.motors = {
            M0: 0,
            M1: 0,
            M2: 0,
            M3: 0
        }
    }

    estop() {
        if (!this.safety.armed) return;

        this.safety.estop = true;
        this.disarm();
    }

    batteryWatchdog() {
        this.A0 = pins.analogReadPin(AnalogPin.P0);
        this.battery_voltage = this.A0 * this.battery_scale_factor;
        this.battery_voltage_smooth = 0.1*this.battery_voltage + 0.9*this.battery_voltage_smooth;
        this.battery_level = (this.battery_voltage_smooth - 3.4)/0.8;
        this.safety.low_battery = (this.battery_level <= 0);
        this.checkCharging();

        if (this.charging) {
            this.safety.low_battery = false;

            if (this.safety.armed) {
                this.disarm();
            }
        }
        
        if (this.safety.low_battery) {
            this.disarm();
            return;
        }
    }

    readSensors() {
        this.imu.accelerometer.read();
        this.imu.gyroscope.read();
        this.calculateAngles();
    }

    update() {
        if (this.safety.estop || !this.safety.armed) return;
        this.safety.crashed = this.safety.armed && ((Math.abs(this.imu.gyroscope.angles.roll) > 90) || (Math.abs(this.imu.gyroscope.angles.pitch) > 90));

        if (this.safety.crashed) {
            this.estop();
            return;
        }

        if (this.pid.active) {
            this.pid.update(this);
        }

        this.pca.writeMotors(this.motors);
    }

    calculateAngles() {
        let dt = 1e-6 * this.physics_timer.get_delta();
        this.imu.accelerometer.angles.pitch = -57.295*Math.atan2(this.imu.accelerometer.reading.y, this.imu.accelerometer.reading.z) - this.imu.accelerometer.reference.pitch;
        this.imu.accelerometer.angles.roll = -57.295*Math.atan2(this.imu.accelerometer.reading.x, this.imu.accelerometer.reading.z) - this.imu.accelerometer.reference.roll;

        let temp = -0.00762939*dt;
        this.imu.gyroscope.delta.set(this.imu.gyroscope.reading);
        this.imu.gyroscope.delta.iSub(this.imu.gyroscope.reference);
        this.imu.gyroscope.delta.iScale(temp);
        this.imu.gyroscope.delta.y *= -1;

        this.imu.gyroscope.angles.pitch = 0.01*this.imu.accelerometer.angles.pitch + 0.99*(this.imu.gyroscope.delta.x + this.imu.gyroscope.angles.pitch);
        this.imu.gyroscope.angles.roll = 0.01*this.imu.accelerometer.angles.roll + 0.99*(this.imu.gyroscope.delta.y + this.imu.gyroscope.angles.roll);

        this.imu.gyroscope.angles.yaw += this.imu.gyroscope.delta.z;
    }

    checkCharging() {
        if (this.A0 == 0) {
            this.charging = false;
            this.charged = false;
            basic.showString('B');
        } else if (this.A0 > 780) {
            if (this.A0 > 950) {
                this.charging = false;
                basic.showString("Charged");
            } else {
                this.charging = true;

                basic.showLeds(`
                    . . # . .
                    . # . # .
                    . # . # .
                    . # . # .
                    . # # # .
                `);

                basic.showLeds(`
                    . . # . .
                    . # . # .
                    . # . # .
                    . # # # .
                    . # # # .
                `);

                basic.showLeds(`
                    . . # . .
                    . # . # .
                    . # # # .
                    . # # # .
                    . # # # .
                `);

                basic.showLeds(`
                    . . # . .
                    . # # # .
                    . # # # .
                    . # # # .
                    . # # # .
                `);
            }
        } else {
            this.charging = false;
        }
    }
};

//% color="#caac19" weight=100 block="Air:Bit"
namespace AirBit {
    let drone: Drone;
    
    //% block="init()"
    //% group="Setup"
    export function init() {
        drone = new Drone();

        basic.forever(() => {drone.batteryWatchdog();});
        basic.forever(() => {drone.update();});
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

    //% block="gyroPitch()"
    //% group="Telemetry"
    export function gyroPitch(): number {
        return drone.imu.gyroscope.angles.pitch;
    }

    //% block="gyroRoll()"
    //% group="Telemetry"
    export function gyroRoll(): number {
        return drone.imu.gyroscope.angles.roll;
    }

    //% block="gyroYaw()"
    //% group="Telemetry"
    export function gyroYaw(): number {
        return drone.imu.gyroscope.angles.yaw;
    }

    //% block="accelX()"
    //% group="Telemetry"
    export function accelX(): number {
        return drone.imu.accelerometer.reading.x;
    }

    //% block="accelY()"
    //% group="Telemetry"
    export function accelY(): number {
        return drone.imu.accelerometer.reading.y;
    }

    //% block="accelZ()"
    //% group="Telemetry"
    export function accelZ(): number {
        return drone.imu.accelerometer.reading.z;
    }

    //% block="armed()"
    //% group="State"
    export function armed(): boolean {
        return drone.safety.armed;
    }

    //% block="crashed()"
    //% group="State"
    export function crashed(): boolean {
        return drone.safety.crashed;
    }

    //% block="emergencyStopped()"
    //% group="State"
    export function emergencyStopped(): boolean {
        return drone.safety.estop;
    }

    //% block="setWifiChannel($channel)"
    //% group="Setup"
    export function setWifiChannel(channel: number) {
        radio.setGroup(channel);
    }

    //% block="initialised()"
    //% group="State"
    export function initialised(): boolean {
        return (drone !== undefined) && drone.init_state;
    }

    //% draggableParameters="reporter"
    //% block="onControlReceived($signal $value)"
    //% group="Control"
    export function onControlReceived(signal: ControlSignals, handler: (value?: number) => void) {
        drone.handlers.push({
            name: controlSignalMap[signal],
            func: handler
        });
    }

    //% block="setThrottle($amount)"
    //% group="Control"
    export function setThrottle(amount: number) {
        drone.controls.throttle = amount;

        if (!drone.pid.active) {
            drone.motors.M0 = amount;
            drone.motors.M1 = amount;
            drone.motors.M2 = amount;
            drone.motors.M3 = amount;
        }
    }

    //% block="changeThrottle($amount)"
    //% group="Control"
    export function changeThrottle(amount: number) {
        drone.controls.throttle += amount;

        if (!drone.pid.active) {
            drone.motors.M0 += amount;
            drone.motors.M1 += amount;
            drone.motors.M2 += amount;
            drone.motors.M3 += amount;
        }
    }

    //% block="setPitch($amount)"
    //% group="Control"
    export function setPitch(amount: number) {
        drone.controls.pitch = amount;
    }

    //% block="setRoll($amount)"
    //% group="Control"
    export function setRoll(amount: number) {
        drone.controls.roll = amount;
    }

    //% block="setYaw($amount)"
    //% group="Control"
    export function setYaw(amount: number) {
        drone.controls.yaw = amount;
    }

    //% block="throttle()"
    //% group="State"
    export function throttle(): number {
        return drone.controls.throttle;
    }

    //% block="calibrate($samples)"
    //% group="Setup"
    //% samples.defl=500
    export function calibrate(samples: number) {
        drone.imu.gyroscope.calibrate(samples);
    }

    //% block="enableControls($enabled)"
    //% group="Control"
    //% enabled.defl=true
    export function enableControls(enabled: boolean) {
        if (enabled) {
            radio.onReceivedBuffer(drone.controls.loadPacket);
        } else {
            radio.onReceivedBuffer((buffer: Buffer) => {});
        }
    }

    //% block="enableStabilisation($enabled)"
    //% group="Setup"
    //% enabled.defl=true
    export function enableStabilisation(enabled: boolean) {
        drone.pid.active = enabled;
    }

    //% block="setP($value, $axis)"
    //% group="Advanced"
    export function setP(value: number, axis: PIDAxis) {
        switch (axis) {
            case PIDAxis.rollPitch:
                drone.pid.roll_pitch_p = value;
                break;

            case PIDAxis.yaw:
                drone.pid.yaw_p = value;
                break;
        }
    }

    //% block="setI($value)"
    //% group="Advanced"
    export function setI(value: number) {
        drone.pid.roll_pitch_i = value;
    }

    //% block="setD($value, $axis)"
    //% group="Advanced"
    export function setD(value: number, axis: PIDAxis) {
        switch (axis) {
            case PIDAxis.rollPitch:
                drone.pid.roll_pitch_d = value;
                break;

            case PIDAxis.yaw:
                drone.pid.yaw_d = value;
                break;
        }
    }

    //% block="setMotorThrottles(|  m0$m0|  m1$m1|  m2$m2|  m3$m3)"
    //% group="Control"
    export function setMotorThrottles(m0: number, m1: number, m2: number, m3: number) {
        drone.pca.writeMotors({
            M0: m0, M1: m1, M2: m2, M3: m3
        });
    }

    //% block="readSensors()"
    //% group="Telemetry"
    export function readSensors() {
        drone.readSensors();
    }

    //% block="updateMotors()"
    //% group="Control"
    export function updateMotors() {
        drone.update();
    }

    //% block="getThrottleFor(motor$motor)"
    //% group="Telemetry"
    export function getThrottleFor(motor: MotorLabels): number {
        switch (motor) {
            case MotorLabels.M0:
                return drone.motors.M0;
                break;
            
            case MotorLabels.M1:
                return drone.motors.M1;
                break;

            case MotorLabels.M2:
                return drone.motors.M2;
                break;

            case MotorLabels.M3:
                return drone.motors.M3;
                break;
        }
    }
}